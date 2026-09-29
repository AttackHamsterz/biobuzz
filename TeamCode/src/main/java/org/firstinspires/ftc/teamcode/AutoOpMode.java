package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.pedropathing.ivy.Scheduler;

@Autonomous(name = "Auto", group = "Robot")
@Disabled

public abstract class AutoOpMode extends StandardSetupOpMode {

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
        buildSchedule();
    }

    @Override
    public void loop() {
        // Clear the bulk cache so the swerve pod encoders read fresh values
        super.loop();
        motion.follower.update();
        Scheduler.execute();

        motion.getTelemetry(telemetry);
        telemetry.addData("Following", motion.follower.following());
        telemetry.addData("Path Completion", motion.follower.completion());
        telemetry.update();
    }

}
