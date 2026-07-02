package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
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

    public static CoaxialPod leftBack(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "leftBackMotor",
                "leftBackServo",
                "leftBackEncoder",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                Math.toRadians(60),
                new Pose(-2, -2),
                0,
                3.3,
                false);
    }

    public static CoaxialPod rightBack(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "rightBackMotor",
                "rightBackServo",
                "rightBackEncoder",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                Math.toRadians(-60),
                new Pose(-2, 2),
                0,
                3.3,
                false);
    }

    public static CoaxialPod centerFront(HardwareMap hardwareMap) {
        return new CoaxialPod(
                hardwareMap,
                "centerFrontMotor",
                "centerFrontServo",
                "centerFrontEncoder",
                new PIDFCoefficients( 0.3, 0, 0.005, 0.01),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                Math.toRadians(180),
                new Pose(2, 0),
                0,
                3.3,
                false);
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants,
                    leftBack(hardwareMap),
                    rightBack(hardwareMap),
                    centerFront(hardwareMap))
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}