package org.firstinspires.ftc.teamcode.pedroPathing; // Use your actual package name

import android.os.Handler;
import android.os.Looper;

import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.drivetrains.SwervePod;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import com.pedropathing.control.PIDFCoefficients;

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
    private static final double MOTOR_CACHING_THRESHOLD = 0.01;
    private static final double SERVO_CACHING_THRESHOLD = 0.01;

    private final String prefix;

    private final DcMotorEx motor;
    private final CRServo servo;
    private final AnalogInput encoder;

    private final PIDFController turnPID;
    private final Pose offset;

    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final double zeroVoltage;
    private double startAngleFraction;

    private double lastDrivePower = 0;
    private double lastTurnPower = 0;

    // Replicate the constructor you use in Constants.java
    public GearedCoaxialPod(HardwareMap hardwareMap, String prefix, PIDFCoefficients turnPIDFCoefficients,
                            DcMotorSimple.Direction driveDirection, CRServo.Direction servoDirection,
                            Pose podOffset, double zeroVoltage, double analogMinVoltage, double analogMaxVoltage) {
        this.prefix = prefix;
        motor = hardwareMap.get(DcMotorEx.class, prefix + "Motor");
        servo = hardwareMap.get(CRServo.class, prefix + "Servo");
        encoder = hardwareMap.get(AnalogInput.class, prefix + "Encoder");
        turnPID = new PIDFController(turnPIDFCoefficients);
        this.offset = podOffset;
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.zeroVoltage = zeroVoltage;

        motor.setDirection(driveDirection);
        servo.setDirection(servoDirection);
        setToFloat();

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
        double errorVoltage = currentVoltage - zeroVoltage;
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
    public Pose getOffset() {
        return offset;
    }

    @Override
    public double getAngle() {
        // Encoder position after zeroing gives us our angle
        int encoderCount = motor.getCurrentPosition();
        return ((((double)encoderCount * ELC_SCALE_FACTOR + startAngleFraction) + 1.0) % 1.0) * TWO_PI;
    }

    @Override
    public double adjustThetaForEncoder(double wheelTheta) {
        // Our encoder assumes forward is 0 radians rotating counter clockwise
        // Pedro pathing angles are 0 degrees right rotating counter clockwise
        //return MathFunctions.normalizeAngle(wheelTheta - Math.PI / 2.0);

        // wheelTheta is in radians. If encoder is reversed, use wheelTheta directly; otherwise invert.
        //if encoder is reversed, ccw (top down) is positive, if unreversed than cw is positive
        double t = 2 * Math.PI - wheelTheta;
        // servo zero offset: +90 degrees -> +pi/2 radians
        t += Math.PI / 2.0;
        return MathFunctions.normalizeAngle(t);
    }

    @Override
    public void move(double targetAngleRad, double drivePower, boolean ignoreAngleChanges) {
        // Convert hardware angle to radians and normalize
        double actualRad = getAngle();
        actualRad = MathFunctions.normalizeAngle(actualRad);
        double desiredRad = adjustThetaForEncoder(targetAngleRad);

        // Shortest-path error in radians (signed)
        double mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
        double dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
        double signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;

        // PID uses radians (tune PIDF for radian error)
        double errorRad = signedRad;

        // Minimize rotation: flip + invert drive if > 90°
        if (Math.abs(errorRad) > (Math.PI / 2.0)) {
            // add 180 degrees (pi radians)
            desiredRad = MathFunctions.normalizeAngle(desiredRad + Math.PI);
            drivePower = -drivePower;

            // recompute signed error
            mag = MathFunctions.getSmallestAngleDifference(actualRad, desiredRad);
            dir = MathFunctions.getTurnDirection(actualRad, desiredRad);
            signedRad = (mag == Math.PI) ? -Math.PI : mag * dir;
            errorRad = signedRad;
        }

        // Setpoint close to current so PID follows shortest path
        double setpointRad = actualRad + errorRad;

        if (Math.abs(errorRad) < (2.0 * Math.PI / 180.0)) {
            turnPID.updateFeedForwardInput(0);
        } else {
            turnPID.updateFeedForwardInput(MathFunctions.getTurnDirection(actualRad, desiredRad));
        }

        turnPID.updateError(setpointRad - actualRad);
        double turnPower = MathFunctions.clamp(turnPID.run(), -1.0, 1.0);

        // please don't change the next 5 lines took like 5 hours to figure ts out
        if (ignoreAngleChanges) {
            lastTurnPower = 0;
            servo.setPower(0);
        } else if (Math.abs(turnPower - lastTurnPower) > SERVO_CACHING_THRESHOLD || (turnPower == 0 && lastTurnPower != 0)) {
            lastTurnPower = turnPower;
            servo.setPower(turnPower);
        }

        if (Math.abs(drivePower - lastDrivePower) > MOTOR_CACHING_THRESHOLD || (drivePower == 0 && lastDrivePower != 0)) {
            lastDrivePower = drivePower;
            motor.setPower(drivePower);
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
    public String debugString() {
        double angle = getAngle();
        return prefix + " {"
                + "\ncurrent angle = " + Math.toDegrees(angle)
                + "\nservo Power = " + servo.getPower()
                + "\ndrive Power = " + motor.getPower()
                + "\n}";
    }
}