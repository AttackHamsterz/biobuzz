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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static final double TESTING_POWER_LIMIT = 1.0;

    public static final double FRONT_LEFT_ZERO = 2.466;
    public static final double FRONT_LEFT_MIN = 0.013;
    public static final double FRONT_LEFT_MAX = 3.225;

    public static final double FRONT_RIGHT_ZERO = 1.062;
    public static final double FRONT_RIGHT_MIN = 0.014;
    public static final double FRONT_RIGHT_MAX = 3.229;

    public static final double BACK_LEFT_ZERO = 2.361;
    public static final double BACK_LEFT_MIN = 0.015;
    public static final double BACK_LEFT_MAX = 3.222;

    public static final double BACK_RIGHT_ZERO = 1.299;
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
                c.xPodOffset.set(4.170056140328955);
                c.yPodOffset.set(2.2461595310000924);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
                c.globalDistanceUnit.set(DistanceUnit.INCH);
                c.offsetUnits.set(DistanceUnit.INCH);
            }
    );

    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
                c.staticFrictionCoefficient.set(FRICTION_COEF);
                c.epsilon.set(0.0175); // Only allow about a degree of error
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            c.headingDriveRatio.set(0.3);
            c.cosineScale.set(true);

            c.maxAchievableForwardVelocity.set(77.54027009174605);
            c.maxAchievableStrafeVelocity.set(77.54027009174605);
            c.naturalForwardDeceleration.set(38.049105010661386);
            c.naturalStrafeDeceleration.set(38.049105010661386);
            c.maxDecelerationConstraint.set(38.049105010661386);

            double lateralScale = 0.2;
            Controller primaryTranslationalForward = Controller.proportional(0.2326085417794935);
            Controller secondaryTranslationalForward = Controller.proportional(0.08594264077281558);
            Controller primaryTranslationalLateral = Controller.proportional(0.2326085417794935*lateralScale);
            Controller secondaryTranslationalLateral = Controller.proportional(0.08594264077281558*lateralScale);
            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
            c.coast.set(Controller.proportionalFeedforward(0.011878192675601161));
            c.brake.set(Controller.proportionalFeedforward(0.010096463774260987));

            double headingScale = 0.2;
            c.headingFeedback.set(Controller.proportional(10*headingScale));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.17461910688461735*headingScale, 0.07330594236272889*headingScale));
            c.linearBrakeCoefficients.set(Matrix.diag(0.094687708786002, 0.094687708786002));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.012229758832106856, 0.012229758832106856));
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
        return new HamFollower(localizer, swerve, foresight);
    }
}