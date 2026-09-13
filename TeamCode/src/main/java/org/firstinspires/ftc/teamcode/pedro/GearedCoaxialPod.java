package org.firstinspires.ftc.teamcode.pedro; // Use your actual package name

import android.os.Handler;
import android.os.Looper;

import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwervePod;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

/**
 * Implements a coaxial swerve pod with a geared servo for increased torque
 * The pods drive bevel gears should all be on the left side of the robot
 * CCW (forward) drive motor should roll the robot forward
 * CCW (forward) servo motor should turn the wheel left
 */
public class GearedCoaxialPod implements SwervePod {
    private static final double TWO_PI = Math.PI * 2.0;
    private static final double ENCODER_TICS_PER_REV = 4000.0;
    private static final double ENCODER_GEAR_RATIO = 7.0;
    private static final double ELC_SCALE_FACTOR = 1.0 / (ENCODER_TICS_PER_REV * ENCODER_GEAR_RATIO);

    private final String name;
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AnalogInput encoder;

    private final Controller turnController;
    private final Vector2D podOffset;

    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final double analogZeroVoltage;
    private double startAngleFraction;

    private final boolean encoderReversed;
    private final double servoCachingThreshold;
    private final double motorCachingThreshold;
    private double lastDrivePower = 0;
    private double lastTurnPower = 0;
    private final boolean flipDigitalEncoder;

    // Replicate the constructor you use in Constants.java
    public GearedCoaxialPod(HardwareMap hardwareMap, CoaxialPodConfig config)
    {
        name = config.name.get();
        motor = hardwareMap.get(DcMotorEx.class, name + "Motor");
        servo = hardwareMap.get(CRServo.class, name + "Servo");
        encoder = hardwareMap.get(AnalogInput.class, name + "Encoder");
        turnController = config.turnController.get();
        podOffset = config.podOffset.get();
        analogMinVoltage = config.analogMinVoltage.get();
        analogMaxVoltage = config.analogMaxVoltage.get();
        analogZeroVoltage = config.angleOffsetRad.get();
        encoderReversed = config.encoderReversed.get();
        servoCachingThreshold = config.servoCachingThreshold.get();
        motorCachingThreshold = config.motorCachingThreshold.get();

        flipDigitalEncoder = config.driveDirection.get() == DcMotorSimple.Direction.REVERSE;
        motor.setDirection(config.driveDirection.get());
        servo.setDirection(config.servoDirection.get());
        setToFloat();
        servo.setPower(0);

        // Final step: init zero angle after hardware is stable (1000ms)
        new Handler(Looper.getMainLooper()).postDelayed(this::init, 1000);
    }

    /**
     * Initialize the pod by comparing the current analog voltage to the expected voltage
     */
    public void init(){
        // Current pod voltage
        double currentVoltage = encoder.getVoltage();

        // Determine how far away from our zero angle orientation voltage
        double errorVoltage = currentVoltage - analogZeroVoltage;
        errorVoltage = errorVoltage % 3.3;
        if (errorVoltage > 1.65)
            errorVoltage -= 3.3;
        else if (errorVoltage < -1.65)
            errorVoltage += 3.3;

        // Convert voltage error to start angle fraction
        startAngleFraction = errorVoltage / (analogMaxVoltage - analogMinVoltage) / ENCODER_GEAR_RATIO;

        // Reset the encoder now that we know the angle offset from zero
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public String name() {
        return name;
    }

    @Override
    public Vector2D getOffset() {
        return podOffset;
    }

    @Override
    public double getAngle() {
        // Encoder position after zeroing gives us our angle
        int encoderCount = flipDigitalEncoder ? -motor.getCurrentPosition() : motor.getCurrentPosition();
        return ((((double)encoderCount * ELC_SCALE_FACTOR + startAngleFraction) + 1.0) % 1.0) * TWO_PI;
    }

    /**
     * Sets turn servo power in [-1, 1].
     * @param power turn servo power
     */
    public void setServoPower(double power) {
        lastTurnPower = power;
        servo.setPower(power);
    }

    /**
     * Sets drive motor power in [-1, 1].
     * @param power drive motor power
     */
    public void setMotorPower(double power) {
        lastDrivePower = power;
        motor.setPower(power);
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // Our encoder assumes forward is 0 radians rotating counter clockwise
        // Pedro pathing angles are 0 degrees right rotating counter clockwise
        //return MathFunctions.normalizeAngle(wheelTheta - Math.PI / 2.0);

        // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double t = encoderReversed ? wheelTheta : (2 * Math.PI - wheelTheta);
        // servo zero offset: +90 degrees -> +pi/2 radians
        t += Math.PI / 2.0;
        return Angle.normalize(t);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        // Convert hardware angle to radians and normalize
        double actualRad = getAngle();
        actualRad = Angle.normalize(actualRad);
        double desiredRad = adjustThetaForEncoder(targetAngleRad);

        // Shortest-path error in radians (signed)
        double mag = Angle.smallestDifference(actualRad, desiredRad);
        double dir = Angle.turnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // PID uses radians (tune PIDF for radian error)
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90°
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            // add 180 degrees (pi radians)
            desiredRad = Angle.normalize(desiredRad + Math.PI);
            drivePower = -drivePower;

            // recompute signed error
            mag = Angle.smallestDifference(actualRad, desiredRad);
            dir = Angle.turnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        // Setpoint close to current so PID follows shortest path
        double setpointRad = actualRad + errorRad;
        double turnPower;
        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPower = Utils.clamp(
                    turnController.calculate(0, errorRad),
                    -1.0,
                    1.0);
        } else {
            turnPower = Utils.clamp(
                    turnController.calculate(Angle.turnDirection(actualRad, desiredRad), errorRad),
                    -1.0, 1.0
            );
        }

        // Avoid overshoot
        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            servo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > servoCachingThreshold || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            servo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > motorCachingThreshold || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            //motor.setPower(drivePower);
        }
    }

    @Override
    public void setToFloat() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void setToBreak() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = new HashMap<>();
        map.put("servoName", name);
        map.put("angleAfterOffsetDeg", Math.toDegrees(getAngle()));
        map.put("servoPower", servo.getPower());
        map.put("drivePower", motor.getPower());
        return map;
    }
}