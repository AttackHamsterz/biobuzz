package org.firstinspires.ftc.teamcode.pedroPathing; // Use your actual package name

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.control.PIDFCoefficients;

public class GearedCoaxialPod extends CoaxialPod {

    // Define your encoder gear ratio constant
    private static final double ENCODER_GEAR_RATIO = 7.0;
    private static final double ENCODER_WRAP_AMOUNT = 2.0 * Math.PI / ENCODER_GEAR_RATIO;

    // Wrap variables
    private double lastRawAngle = 0.0;
    private int wrapCount = 0;
    private boolean firstRead = true;

    // Replicate the constructor you use in Constants.java
    public GearedCoaxialPod(HardwareMap hardwareMap, String motorName, String servoName,
                            String turnEncoderName, PIDFCoefficients turnPIDFCoefficients,
                            DcMotorSimple.Direction driveDirection, CRServo.Direction servoDirection,
                            double angleOffsetRad, Pose podOffset, double analogMinVoltage, double analogMaxVoltage,
                            boolean encoderReversed) {
        super(hardwareMap, motorName, servoName, turnEncoderName, turnPIDFCoefficients, driveDirection, servoDirection, angleOffsetRad, podOffset, analogMinVoltage, analogMaxVoltage, encoderReversed);
    }

    /**
     * Overriding the method Pedro Pathing uses to read the raw angle.
     * We grab the standard calculated raw angle,
     * apply our 7:1 reduction, and wrap it cleanly into [0, 2π].
     */
    @Override
    public double getRawAngleRad() {
        // 1. Get the raw total angle from the base class implementation (which returns [0, 2π])
        double rawAngle = super.getAngle();

        // Determine wrap amount
        if (firstRead) {
            lastRawAngle = rawAngle;
            firstRead = false;
        }
        double delta = rawAngle - lastRawAngle;
        if (delta < -Math.PI) {
            wrapCount++;
        } else if (delta > Math.PI) {
            wrapCount--;
        }
        lastRawAngle = rawAngle;

        // 2. Divide raw by ENCODER_GEAR_RATIO and add wrap
        double adjustedAngle = rawAngle / ENCODER_GEAR_RATIO + (double)wrapCount * ENCODER_WRAP_AMOUNT;

        // 3. Keep the resulting angle normalized between 0 and 2π
        return (adjustedAngle + (2.0 * Math.PI)) % (2.0 * Math.PI);
    }
}