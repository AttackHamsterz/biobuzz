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
            //.useSecondaryDrivePIDF(true)
            //.useSecondaryHeadingPIDF(true)
            //.useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.008, 0))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0825, 0, 0.008, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.75, 0, 0.003, 0))
            //.secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.015, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.00003, 0.6, 0.13))
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0, 0.000002, 0.6, 0.13))
            .centripetalScaling(0.0005)
            .mass(5);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(0)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .maxPower(1.0)
            .velocity(120.0)
            .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES)
            .staticFrictionCoefficient(0.05)
            .useBrakeModeInTeleOp(true);

    public static final double FRONT_LEFT_ZERO = 2.537;
    public static final double FRONT_RIGHT_ZERO = 2.225;
    public static final double BACK_LEFT_ZERO = 0.053;
    public static final double BACK_RIGHT_ZERO = 2.932;

    public static final double FRONT_LEFT_MIN = 0.017;
    public static final double FRONT_LEFT_MAX = 3.23;
    public static final double FRONT_RIGHT_MIN = 0.019;
    public static final double FRONT_RIGHT_MAX = 3.224;
    public static final double BACK_LEFT_MIN = 0.015;
    public static final double BACK_LEFT_MAX = 3.227;
    public static final double BACK_RIGHT_MIN = 0.012;
    public static final double BACK_RIGHT_MAX = 3.228;

    public static GearedCoaxialPod frontLeft(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "frontLeft",
                new PIDFCoefficients( 0.6, 0, 0.005, 0),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                new Pose(6.75, -6.75),
                FRONT_LEFT_ZERO,
                FRONT_LEFT_MIN,
                FRONT_LEFT_MAX);
    }

    public static GearedCoaxialPod frontRight(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "frontRight",
                new PIDFCoefficients( 0.6, 0, 0.005, 0),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                new Pose(6.75, 6.75),
                FRONT_RIGHT_ZERO,
                FRONT_RIGHT_MIN,
                FRONT_RIGHT_MAX);
    }

    public static GearedCoaxialPod backLeft(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "backLeft",
                new PIDFCoefficients( 0.6, 0, 0.005, 0),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                new Pose(-6.75, -6.75),
                BACK_LEFT_ZERO,
                BACK_LEFT_MIN,
                BACK_LEFT_MAX);
    }

    public static GearedCoaxialPod backRight(HardwareMap hardwareMap) {
        return new GearedCoaxialPod(
                hardwareMap,
                "backRight",
                new PIDFCoefficients( 0.6, 0, 0.005, 0),
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                new Pose(-6.75, 6.75),
                BACK_RIGHT_ZERO,
                BACK_RIGHT_MIN,
                BACK_RIGHT_MAX);
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