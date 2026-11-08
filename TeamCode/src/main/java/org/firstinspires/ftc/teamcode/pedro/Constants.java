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

    // Smallest drive power that actually moves the robot (static friction). Real drive commands
    // are lifted to at least this so a nearly-finished correction can't stall. 0 = off.
    // Measured breakaway was about 0.1 in Swerve Calibration; 0.08 (80% of that) was the old value.
    // Turned off 2026-11-01 to rule it out while debugging localization and spin drift.
    public static final double STATIC_DRIVE_POWER = 0.0;

    // Auto only (HamSwerve): drive power ramps from 0 when the pods average this many degrees off
    // their target angle up to full power when aligned, so the robot doesn't push the wrong way
    // while the pods are still turning. 2026-11-06: added for side-to-side steering oscillation
    // near the end of paths. 90 gives back the old plain cos scaling exactly.
    public static final double AUTO_ALIGN_GATE_DEG = 30.0;

    // 2026-11-08 - Cal
    public static final double FRONT_LEFT_ZERO = 2.409;
    public static final double FRONT_LEFT_MIN = 0.013;
    public static final double FRONT_LEFT_MAX = 3.225;

    public static final double FRONT_RIGHT_ZERO = 1.118;
    public static final double FRONT_RIGHT_MIN = 0.014;
    public static final double FRONT_RIGHT_MAX = 3.229;

    public static final double BACK_LEFT_ZERO = 1.174;
    public static final double BACK_LEFT_MIN = 0.015;
    public static final double BACK_LEFT_MAX = 3.222;

    public static final double BACK_RIGHT_ZERO = 1.307;
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
                // Measured 2026-11-01. Both pods are at the back: the forward pod is 3.875" right
                // of center, the strafe pod 8.233" behind. Pinpoint wants left and forward as
                // positive, so both are negative. Stick-up drives away from the pods, confirming
                // the code's front is the team's front. Spin test passed (within ~1.6" at 180 deg).
                c.xPodOffset.set(-3.875);
                c.yPodOffset.set(-8.233);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                // REVERSED 2026-11-01: strafing right raised y (Pedro's +y is left), and a 180 deg spin
                // was off by 2 x 16.5", both of which match a backward strafe pod.
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

    // Foresight gains. Kept as named constants (not locals inside foresightConfig) so RunLog can
    // write them into every log's settings header; Pedro's Controller objects can't be read back.
    // Moved out of the lambda 2026-11-06 with no value changes.
    public static final double TRANSLATIONAL_KP = 0.2326085417794935;
    public static final double DRIVE_SCALE = 1.0; // was 2.0 (2026-11-06: over-corrected on the Line Test)
    // was 0.18 while the strafe pod read backward (fixed 2026-11-01); 1.0 (flat kP 0.23) weaved
    // 5-6 in side to side, worst near the target, so halved 2026-11-06.
    public static final double LATERAL_SCALE = 0.5;
    // 2026-11-06: lateral D to damp the side-to-side steering oscillation. Pedro's PIDController
    // takes d(error)/dt per SECOND (nanoTime), error in inches, so D is power per in/s.
    // Foresight already damps a little: the error uses pose + brake displacement, which works
    // out to about kP * linearBrake (0.116 * 0.0947 = ~0.011 per in/s). 0.01 roughly doubles it.
    // 0.01 held the line much better but swung the pods 3-4x/s; 0.005 went off course much
    // more for little gain, so back to 0.01. RunLog (bulk cached, 28 Hz) showed the strafe
    // command flipping cap to cap ~3x/s on +-0.3 in wobbles, driven by the speed terms
    // (brake prediction ~0.25 at 10 in/s + D ~0.1), so D off; brake prediction alone damps.
    public static final double LATERAL_D = 0.0;
    // 2026-11-06: flat gain on both axes. The old piecewise step to a secondary gain
    // (0.08594264077281558) under 2.5 in was a ~2.7x jump and made the Line Test over-correct.
    // Largest power each path-correction axis may ask for (1.0 = no cap). Foresight gives
    // correction priority over driving, so an uncapped correction can take all the power.
    public static final double CORRECTION_CAP = 0.5;
    // 2026-11-06: lateral capped lower so the correction eases back instead of snapping. On
    // swerve the correction steers the wheels; 0.2 against normal drive power is ~14 deg max.
    // Hold uses the same controller; 0.2 is still above the ~0.1 breakaway power.
    public static final double LATERAL_CORRECTION_CAP = 0.2;
    public static final double COAST_FF = 0.011878192675601161;
    public static final double BRAKE_FF = 0.010096463774260987;
    // 2026-11-06: was 0.004 (kP 0.04 -> HamSwerve epsilon zeroed heading errors under ~25 deg).
    // 0.05 (kP 0.5) held heading but overshot; halved.
    public static final double HEADING_GAIN_SCALE = 0.025; // kP 0.25 (headingFeedback is 10x this)
    public static final double HEADING_BRAKE_SCALE = 0.004;

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            c.headingDriveRatio.set(0.3);
            c.cosineScale.set(true);

            c.maxAchievableForwardVelocity.set(82.357);
            c.maxAchievableStrafeVelocity.set(82.357);
            c.naturalForwardDeceleration.set(52.0);
            c.naturalStrafeDeceleration.set(52.0);
            c.maxDecelerationConstraint.set(52.0);

            Controller primaryTranslationalForward = Controller.proportional(TRANSLATIONAL_KP*DRIVE_SCALE);
            Controller primaryTranslationalLateral = Controller.pid(TRANSLATIONAL_KP*LATERAL_SCALE, 0, LATERAL_D);
            c.forwardTranslational.set(capped(primaryTranslationalForward, CORRECTION_CAP));
            c.strafeTranslational.set(capped(primaryTranslationalLateral, LATERAL_CORRECTION_CAP));
            c.coast.set(Controller.proportionalFeedforward(COAST_FF));
            c.brake.set(Controller.proportionalFeedforward(BRAKE_FF));

            c.headingFeedback.set(Controller.proportional(10*HEADING_GAIN_SCALE));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.17461910688461735*HEADING_BRAKE_SCALE, 0.07330594236272889*HEADING_BRAKE_SCALE));
            c.linearBrakeCoefficients.set(Matrix.diag(0.094687708786002, 0.094687708786002));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.012229758832106856, 0.012229758832106856));
        }
    );

    private static Controller capped(Controller inner, double cap) {
        return new Controller() {
            @Override
            public double calculate(double target, double error) {
                return Math.max(-cap, Math.min(cap, inner.calculate(target, error)));
            }

            @Override
            public void reset() {
                inner.reset();
            }
        };
    }

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