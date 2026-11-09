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
    // 2026-11-08: back to 0.08. Line Test stalled 1.5 in from the end for 2.3 s at 0.10 drive
    // power with pods aligned, and every leg crept its last ~1.5 in at 0.03-0.10 power.
    public static final double STATIC_DRIVE_POWER = 0.08;

    // Auto only (HamSwerve): drive power ramps from 0 when the pods average this many degrees off
    // their target angle up to full power when aligned, so the robot doesn't push the wrong way
    // while the pods are still turning. 2026-11-06: added for side-to-side steering oscillation
    // near the end of paths. 90 gives back the old plain cos scaling exactly.
    // 2026-11-08: 30 -> 50 -> 90. At 30 the gate cut drive power to 0 on 41-53% of Curve Test loops
    // and 47-80% of holds (stop-start); 30 was chosen while the back-left zero was 18 deg off. 50
    // was faster with similar accuracy. 90 = TeleOp's plain cos scaling, needed with
    // AUTO_BRAKE_MODE: with brakes on, every zero-power cut would be a hard stop.
    public static final double AUTO_ALIGN_GATE_DEG = 90.0;

    // 2026-11-08: brake the drive motors at zero power in auto, like TeleOp (manualBrakeMode).
    // Was FLOAT: the Forward Braking tuner "brakes" by commanding zero power, so it measured
    // COASTING (0.321 v + 0.0103 v^2 = 28 in at 39 in/s). Foresight steers toward that predicted
    // stopping point, so a 28 in lookahead cut corners by up to 7.6 in on the Curve Test.
    // Braking/deceleration constants must be re-measured with the tuner after changing this.
    public static final boolean AUTO_BRAKE_MODE = true;

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
    // Error is in radians and the >90 deg flip caps it, so 0.38 gives ~0.3 turn power at 30 deg off.
    // 2026-11-08: tried 0.7: pods overshot (FR swung -58 -> +82 -> -39 deg in 0.4 s on a straight
    // line), steady-target pod error doubled (median 4.7 -> 9.3 deg). Reverted. The pod loop has
    // little margin at ~30 Hz; tune more pod speed in Swerve Calibration (with TURN_D), not here.
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
    // Every value used to build a Controller or a brake coefficient must live here, not in the lambda.
    //
    // 2026-11-08: retuned after the back-left pod fix and new zero voltages. Translational is
    // piecewise: SECONDARY kP within SECONDARY_DISTANCE_IN of the path, primary kP beyond. Lateral
    // is the forward gain x LATERAL_PENALTY. No correction caps. The 2026-11-06 config (flat kP
    // 0.2326, lateral x0.5, caps 0.5 / 0.2, lateral D 0, heading kP 0.25) is in git at 86d411c.
    public static final double TRANSLATIONAL_KP = 0.2601685739694509;
    public static final double SECONDARY_TRANSLATIONAL_KP = 0.09612533624938191;
    public static final double SECONDARY_DISTANCE_IN = 3.0;
    // 2026-11-08: was 0.01 (lateral kP ~0.003): Line Test drifted to -3.9 in with no correction.
    public static final double LATERAL_PENALTY = 0.25;
    public static final double COAST_FF = 0.011414566942689493;
    public static final double BRAKE_FF = 0.0097023819012860;
    public static final double HEADING_KP_RAW = 7.150330685072329; // before HEADING_PENALTY
    public static final double HEADING_PENALTY = 0.05;
    public static final double HEADING_KP = HEADING_KP_RAW * HEADING_PENALTY; // ~0.358
    // 2026-11-08: re-measured with AUTO_BRAKE_MODE (Heading Braking tuner). Was 0.2210 / 0.07198
    // (coasting): predicted 75 deg of spin to stop from 3 rad/s, now ~26 deg.
    public static final double HEADING_BRAKING_LINEAR = 0.12999448344051004;
    public static final double HEADING_BRAKING_QUADRATIC = 0.0080688711613246;
    // 2026-11-08: re-measured with AUTO_BRAKE_MODE (Forward Braking tuner). Was 0.3214 / 0.01029,
    // measured while auto coasted (FLOAT): 28 in to stop from 39 in/s, now ~7.4 in.
    public static final double FORWARD_BRAKING_LINEAR = 0.11616504904154684;
    public static final double FORWARD_BRAKING_QUADRATIC = 0.0019091966985725823;
    // 2026-11-08: strafe entry of linearBrakeCoefficients split from forward (was 0.321, a copy of
    // the forward value, never measured sideways). At 0.321 the predicted stop point swung +-2 in
    // while actual y swung +-0.6 in, and the lateral correction chased it (strafe kicks, pod swings,
    // align-gate power cuts). 0.0947 is the 2026-11-06 value, the calmest Line Test so far.
    public static final double STRAFE_BRAKING_LINEAR = 0.094687708786002;

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            c.headingDriveRatio.set(0.3);
            c.cosineScale.set(false);
            c.translationalDeviationTolerance.set(5.0);
            //c.headingDeviationTolerance.set(Math.toRadians(25.0));

            c.maxAchievableForwardVelocity.set(72.365);
            c.maxAchievableStrafeVelocity.set(72.365);

            c.naturalForwardDeceleration.set(27.418);
            c.naturalStrafeDeceleration.set(27.418);
            c.maxDecelerationConstraint.set(27.418);

            Controller primaryTranslationalForward = Controller.proportional(TRANSLATIONAL_KP);
            Controller secondaryTranslationalForward = Controller.proportional(SECONDARY_TRANSLATIONAL_KP);
            Controller primaryTranslationalLateral = Controller.proportional(TRANSLATIONAL_KP*LATERAL_PENALTY);
            Controller secondaryTranslationalLateral = Controller.proportional(SECONDARY_TRANSLATIONAL_KP*LATERAL_PENALTY);
            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(SECONDARY_DISTANCE_IN, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(SECONDARY_DISTANCE_IN, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(COAST_FF));
            c.brake.set(Controller.proportionalFeedforward(BRAKE_FF));

            c.headingFeedback.set(Controller.proportional(HEADING_KP));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(HEADING_BRAKING_LINEAR, HEADING_BRAKING_QUADRATIC));
            c.linearBrakeCoefficients.set(Matrix.diag(FORWARD_BRAKING_LINEAR, STRAFE_BRAKING_LINEAR));
            c.quadraticBrakeCoefficients.set(Matrix.diag(FORWARD_BRAKING_QUADRATIC, FORWARD_BRAKING_QUADRATIC));

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
        GuardedLocalizer localizer = new GuardedLocalizer(new PinpointLocalizer(hardwareMap, localizerConfig));
        HamSwerve swerve = new HamSwerve(hardwareMap, driveConfig,
                backLeftPod, frontLeftPod, backRightPod, frontRightPod);
        Foresight foresight = new Foresight(foresightConfig);
        return new HamFollower(localizer, swerve, foresight);
    }
}