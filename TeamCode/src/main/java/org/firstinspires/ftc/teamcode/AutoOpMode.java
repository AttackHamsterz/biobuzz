package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.pedro.RunLog;

@Autonomous(name = "Auto", group = "Robot")
@Disabled

public abstract class AutoOpMode extends StandardSetupOpMode {

    // Per-loop CSV in /sdcard/FIRST/pedrologs (auto_<OpMode>_<time>.csv), same format as the tuning tests
    private RunLog log;

    // Which part of the auto is running, written as the log's leg column so a run can be split
    // apart. Subclasses set it as each step starts (0 = nothing started yet).
    protected int step = 0;

    @Override
    public void init() {
        super.init();
        Scheduler.reset();
    }

    /**
     * Op modes must override the schedule building.
     * Called from start(), so the start pose set during init() is already in place.
     * Ivy starts a scheduled command immediately, so schedule one sequential group
     * rather than several separate commands.
     */
    public abstract void buildSchedule();

    @Override
    public void start() {
        // Don't call super.start(): Motion.start() resets the pose to (0,0,0)
        // and would wipe out the start pose set during init()
        log = new RunLog(motion.follower, "auto_" + getClass().getSimpleName());
        buildSchedule();
    }

    @Override
    public void loop() {
        // Clear the bulk cache so the swerve pod encoders read fresh values
        super.loop();
        motion.follower.update();
        log.row(step);
        Scheduler.execute();

        motion.getTelemetry(telemetry);
        telemetry.addData("Step", step);
        telemetry.addData("Following", motion.follower.following());
        telemetry.addData("Path Completion", motion.follower.completion());
        telemetry.addData("Logging to", log.path);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Flush the log first; stop is called whether auto finished or was cut short
        if (log != null) {
            log.close();
        }
        super.stop();
    }

}
