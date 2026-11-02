package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightSwerveTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    /**
     * How long to wait after building the swerve drivetrain before handing it to a tuner or test.
     *
     * Why this exists: each GearedCoaxialPod finishes its own setup (init()) 1 second after it is
     * constructed, on Android's main thread. Until init() runs, startAngleFraction is still 0, so
     * every pod's angle reading is off by wherever that pod sat when the previous op mode ran its
     * init() (the SDK does not reset motor encoders between op modes).
     *
     * In a normal TeleOp or Auto that never matters: the drivers sit in INIT for more than a second.
     * AutoTune is different. Procedure.runOpMode() calls initOpMode() and then startActiveOpMode()
     * immediately, so waitForStart() returns at once. Without this wait, the first ~1 s of every
     * tuner and every Tests procedure ran with wrong pod angles, and that first second is exactly
     * when the system-ID tuners collect their data.
     *
     * 1.5 s gives 0.5 s of margin over the 1 s delay. It matches the 1.5 s wait that the
     * HeadingBraking and ForwardBraking tuners already use.
     */
    private static final long POD_INIT_SETTLE_MS = 1500;

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightSwerveTuner(
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                Tuning::settledSwerve);
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(Tuning::settledSwerve,
                (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }

    /**
     * Builds the swerve drivetrain and blocks until every pod has run its delayed init().
     * Only AutoTune uses this; Auto and TeleOp build their pods through Constants.create().
     */
    private static Drivetrain settledSwerve(HardwareMap hardwareMap) {
        HamSwerve swerve = new HamSwerve(hardwareMap, Constants.driveConfig,
                new GearedCoaxialPod(hardwareMap, Constants.backLeft),
                new GearedCoaxialPod(hardwareMap, Constants.frontLeft),
                new GearedCoaxialPod(hardwareMap, Constants.backRight),
                new GearedCoaxialPod(hardwareMap, Constants.frontRight));
        try {
            // Sleeping here only pauses the op mode thread. The pods' init() runs on the main
            // thread, so it still fires while we wait.
            Thread.sleep(POD_INIT_SETTLE_MS);
        } catch (InterruptedException e) {
            // The op mode is being stopped. Keep the interrupt flag set so the tuner sees it.
            Thread.currentThread().interrupt();
        }
        return swerve;
    }
}
