package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static final double FRONT_RIGHT_ZERO = 1.103;
    public static final double BACK_RIGHT_ZERO = 1.421;
    public static final double FRONT_LEFT_ZERO = 2.645;
    public static final double BACK_LEFT_ZERO = 2.334;

    public static final double FRONT_RIGHT_MIN = 0.017;
    public static final double FRONT_RIGHT_MAX = 3.23;
    public static final double BACK_RIGHT_MIN = 0.019;
    public static final double BACK_RIGHT_MAX = 3.224;
    public static final double FRONT_LEFT_MIN = 0.015;
    public static final double FRONT_LEFT_MAX = 3.227;
    public static final double BACK_LEFT_MIN = 0.012;
    public static final double BACK_LEFT_MAX = 3.228;

    public static CoaxialPodConfig frontLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("frontLeft");
                c.analogMinVoltage.set(FRONT_LEFT_MIN);
                c.analogMaxVoltage.set(FRONT_LEFT_MAX);
                c.angleOffsetRad.set(FRONT_LEFT_ZERO);
                c.podOffset.set(Vector2D.cartesian(6.75, 6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.turnController.set(
                        Controller.pid(0.6, 0, 0.005)
                                .plus(Controller.proportionalFeedforward(0)));
            }
    );

    public static CoaxialPodConfig frontRight = new CoaxialPodConfig(
            c -> {
                c.name.set("frontRight");
                c.analogMinVoltage.set(FRONT_RIGHT_MIN);
                c.analogMaxVoltage.set(FRONT_RIGHT_MAX);
                c.angleOffsetRad.set(FRONT_RIGHT_ZERO);
                c.podOffset.set(Vector2D.cartesian(6.75, -6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.turnController.set(
                        Controller.pid(0.6, 0, 0.005)
                                .plus(Controller.proportionalFeedforward(0)));
            }
    );

    public static CoaxialPodConfig backLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("backLeft");
                c.analogMinVoltage.set(BACK_LEFT_MIN);
                c.analogMaxVoltage.set(BACK_LEFT_MAX);
                c.angleOffsetRad.set(BACK_LEFT_ZERO);
                c.podOffset.set(Vector2D.cartesian(-6.75, 6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.turnController.set(
                        Controller.pid(0.6, 0, 0.005)
                                .plus(Controller.proportionalFeedforward(0)));
            }
    );

    public static CoaxialPodConfig backRight = new CoaxialPodConfig(
            c -> {
                c.name.set("backRight");
                c.analogMinVoltage.set(BACK_RIGHT_MIN);
                c.analogMaxVoltage.set(BACK_RIGHT_MAX);
                c.angleOffsetRad.set(BACK_RIGHT_ZERO);
                c.podOffset.set(Vector2D.cartesian(-6.75, -6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.turnController.set(
                        Controller.pid(0.6, 0, 0.005)
                                .plus(Controller.proportionalFeedforward(0)));
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(3.6610706960122417);
                c.yPodOffset.set(-7.081870131605254);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );

    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(true);
                c.staticFrictionCoefficient.set(0.0005);
            }
    );

    // TODO - TUNE THIS
    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.3);
            Controller secondaryTranslationalForward = Controller.proportional(0.1);
            Controller primaryTranslationalLateral = Controller.proportional(0.3);
            Controller secondaryTranslationalLateral = Controller.proportional(0.1);
            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
            c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
            c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
            c.headingFeedback.set(Controller.proportional(5.258721785960744));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
            c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
            c.maxAchievableForwardVelocity.set(72.72923108818539);
            c.maxAchievableStrafeVelocity.set(52.34323936525474);
            c.naturalForwardDeceleration.set(85.01144677379789);
            c.naturalStrafeDeceleration.set(104.49787535782846);
        }
    );

    public static Follower create(HardwareMap hardwareMap) {
        GearedCoaxialPod frontLeftPod = new GearedCoaxialPod(hardwareMap, frontLeft);
        GearedCoaxialPod frontRightPod = new GearedCoaxialPod(hardwareMap, frontRight);
        GearedCoaxialPod backLeftPod = new GearedCoaxialPod(hardwareMap, backLeft);
        GearedCoaxialPod backRightPod = new GearedCoaxialPod(hardwareMap, backRight);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        Swerve swerve = new Swerve(hardwareMap, driveConfig,
                backLeftPod, frontLeftPod, backRightPod, frontRightPod);
        Foresight foresight = new Foresight(foresightConfig);
        return new Follower(localizer, swerve, foresight);
    }
}