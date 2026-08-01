package org.firstinspires.ftc.teamcode.pedroPathing; // Use your actual package name

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.control.PIDFCoefficients;

public class GearedCoaxialPod extends CoaxialPod {
    private static final double ENCODER_TICS_PER_REV = 4000.0;
    private static final double ENCODER_GEAR_RATIO = 7.0;
    private static final double ELC_SCALE_FACTOR = 1.0 / (ENCODER_TICS_PER_REV * ENCODER_GEAR_RATIO);
    private final DcMotorEx motor;
    private final AnalogInput encoder;
    private final double analogMinVoltage;
    private final double analogMaxVoltage;
    private final double zeroVoltage;
    private double angleOffsetRad;

    // Replicate the constructor you use in Constants.java
    public GearedCoaxialPod(HardwareMap hardwareMap, String prefix, PIDFCoefficients turnPIDFCoefficients,
                            DcMotorSimple.Direction driveDirection, CRServo.Direction servoDirection,
                            Pose podOffset, double zeroVoltage, double analogMinVoltage, double analogMaxVoltage) {
        super(hardwareMap, prefix + "Motor", prefix + "Servo", prefix + "Encoder", turnPIDFCoefficients, driveDirection, servoDirection, 0, podOffset, analogMinVoltage, analogMaxVoltage, false);
        motor = hardwareMap.get(DcMotorEx.class, prefix + "Motor");
        encoder = hardwareMap.get(AnalogInput.class, prefix + "Encoder");
        this.analogMinVoltage = analogMinVoltage;
        this.analogMaxVoltage = analogMaxVoltage;
        this.zeroVoltage = zeroVoltage;
        this.angleOffsetRad = 0;
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

        // Convert voltage error to angleOffsetRadians, ends up [-pi, pi]
        angleOffsetRad = errorVoltage / (analogMaxVoltage - analogMinVoltage) * 2 * Math.PI;

        // Reset the encoder now that we know the angle offset from zero
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public double getAngleAfterOffsetRad() {
        return getRawAngleRad() - angleOffsetRad;
    }

    /**
     * Overriding offset angle
     * @return offset angle in radians
     */
    @Override
    public double getOffsetAngleRad() {
        double rad = getRawAngleRad() - angleOffsetRad;
        return MathFunctions.normalizeAngle(rad);
    }

    /**
     * Raw angle based on digital encoder, wrap cleanly into [0, 2π].
     */
    @Override
    public double getRawAngleRad() {
        // Encoder position after zeroing gives us our angle
        int encoderCount = motor.getCurrentPosition();

        // Keep the resulting angle normalized between 0 and 2π
        return ((((double)encoderCount * ELC_SCALE_FACTOR) + 1.0) % 1.0) * (2.0 * Math.PI);
    }
}