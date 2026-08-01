package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-197.1)
            .lateralZeroPowerAcceleration(-197.1)
            .useSecondaryDrivePIDF(true).useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.008, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0825, 0, 0.008, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.75, 0, 0.003, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.015, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0, 0.000002, 0.6, 0.13))
            .centripetalScaling(0.0005)
            .mass(5);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(-2)
            .strafePodX(-5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .maxPower(1.0)
            .velocity(74.0)
            .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.X_LOCK)
            .useBrakeModeInTeleOp(true);

    public static GearedCoaxialPod frontLeft(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "frontLeft",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE,
                new Pose(-4.8, 4.8),
                2.263,
                0.026,
                3.23);
    }

    public static GearedCoaxialPod frontRight(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "frontRight",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE,
                new Pose(4.8, 4.8),
                2.512,
                0.017,
                3.225);
    }

    public static GearedCoaxialPod backLeft(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "backLeft",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE,
                new Pose(-4.8, -4.8),
                2.892,
                0.021,
                3.229);
    }

    public static GearedCoaxialPod backRight(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "backRight",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE,
                new Pose(4.8, -4.8),
                0.086,
                0.017,
                3.222);
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants,
                        frontLeft(hardwareMap),
                        frontRight(hardwareMap),
                        backLeft(hardwareMap),
                        backRight(hardwareMap))
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}