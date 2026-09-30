package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static final double TESTING_POWER_LIMIT = 0.5;

    public static final double FRONT_LEFT_ZERO = 2.442;
    public static final double FRONT_LEFT_MIN = 0.013;
    public static final double FRONT_LEFT_MAX = 3.225;

    public static final double FRONT_RIGHT_ZERO = 1.081;
    public static final double FRONT_RIGHT_MIN = 0.014;
    public static final double FRONT_RIGHT_MAX = 3.229;

    public static final double BACK_LEFT_ZERO = 2.367;
    public static final double BACK_LEFT_MIN = 0.015;
    public static final double BACK_LEFT_MAX = 3.222;

    public static final double BACK_RIGHT_ZERO = 1.457;
    public static final double BACK_RIGHT_MIN = 0.02;
    public static final double BACK_RIGHT_MAX = 3.224;

    public static final double FRICTION_COEF = 0.0; //0.0005
    public static final double TURN_P = 0.38;
    public static final double TURN_I = 0.0;
    public static final double TURN_D = 0.018;
    public static final double FL_TURN_F = 0.1;
    public static final double FR_TURN_F = 0.095;;
    public static final double BL_TURN_F = 0.085;
    public static final double BR_TURN_F = 0.1;


    public static CoaxialPodConfig frontLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("frontLeft");
                c.analogMinVoltage.set(FRONT_LEFT_MIN);
                c.analogMaxVoltage.set(FRONT_LEFT_MAX);
                c.angleOffsetRad.set(FRONT_LEFT_ZERO);
                c.podOffset.set(Vector2D.cartesian(-6.75, 6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.turnController.set(
                        Controller.pid(TURN_P, TURN_I, TURN_D)
                                .plus(Controller.proportionalFeedforward(FL_TURN_F)));
            }
    );

    public static CoaxialPodConfig frontRight = new CoaxialPodConfig(
            c -> {
                c.name.set("frontRight");
                c.analogMinVoltage.set(FRONT_RIGHT_MIN);
                c.analogMaxVoltage.set(FRONT_RIGHT_MAX);
                c.angleOffsetRad.set(FRONT_RIGHT_ZERO);
                c.podOffset.set(Vector2D.cartesian(6.75, 6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.turnController.set(
                        Controller.pid(TURN_P, TURN_I, TURN_D)
                                .plus(Controller.proportionalFeedforward(FR_TURN_F)));
            }
    );

    public static CoaxialPodConfig backLeft = new CoaxialPodConfig(
            c -> {
                c.name.set("backLeft");
                c.analogMinVoltage.set(BACK_LEFT_MIN);
                c.analogMaxVoltage.set(BACK_LEFT_MAX);
                c.angleOffsetRad.set(BACK_LEFT_ZERO);
                c.podOffset.set(Vector2D.cartesian(-6.75, -6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.turnController.set(
                        Controller.pid(TURN_P, TURN_I, TURN_D)
                                .plus(Controller.proportionalFeedforward(BL_TURN_F)));
            }
    );

    public static CoaxialPodConfig backRight = new CoaxialPodConfig(
            c -> {
                c.name.set("backRight");
                c.analogMinVoltage.set(BACK_RIGHT_MIN);
                c.analogMaxVoltage.set(BACK_RIGHT_MAX);
                c.angleOffsetRad.set(BACK_RIGHT_ZERO);
                c.podOffset.set(Vector2D.cartesian(6.75, -6.75));
                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.turnController.set(
                        Controller.pid(TURN_P, TURN_I, TURN_D)
                                .plus(Controller.proportionalFeedforward(BR_TURN_F)));
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
                c.xPodOffset.set(4.25);
                c.yPodOffset.set(-2.48);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
                c.staticFrictionCoefficient.set(FRICTION_COEF);
            }
    );

    // TODO - TUNE THIS
    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.24777729225346273);
            Controller secondaryTranslationalForward = Controller.proportional(0.09154708875646973);
            Controller primaryTranslationalLateral = Controller.proportional(0.24777729225346273);
            Controller secondaryTranslationalLateral = Controller.proportional(0.09154708875646973);
            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
            c.coast.set(Controller.proportionalFeedforward(0.01244254832154185));
            c.brake.set(Controller.proportionalFeedforward(0.010576166073310573));

            // BUSTED RIGHT NOW
            c.headingFeedback.set(Controller.proportional(5.258721785960744));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
            c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

            c.maxAchievableForwardVelocity.set(72.2215);
            c.maxAchievableStrafeVelocity.set(72.2215);
            c.naturalForwardDeceleration.set(44.575);
            c.naturalStrafeDeceleration.set(44.575);
        }
    );

    public static Follower create(HardwareMap hardwareMap) {
        GearedCoaxialPod frontLeftPod = new GearedCoaxialPod(hardwareMap, frontLeft);
        GearedCoaxialPod frontRightPod = new GearedCoaxialPod(hardwareMap, frontRight);
        GearedCoaxialPod backLeftPod = new GearedCoaxialPod(hardwareMap, backLeft);
        GearedCoaxialPod backRightPod = new GearedCoaxialPod(hardwareMap, backRight);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, localizerConfig);
        HamSwerve swerve = new HamSwerve(hardwareMap, driveConfig,
                backLeftPod, frontLeftPod, backRightPod, frontRightPod);
        Foresight foresight = new Foresight(foresightConfig);
        return new Follower(localizer, swerve, foresight);
    }
}