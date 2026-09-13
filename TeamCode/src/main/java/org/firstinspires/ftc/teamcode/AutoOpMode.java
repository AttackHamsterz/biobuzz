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
     * Op modes must override the schedule building
     */
    public abstract void buildSchedule();

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        motion.follower.update();
        Scheduler.execute();
    }

}
