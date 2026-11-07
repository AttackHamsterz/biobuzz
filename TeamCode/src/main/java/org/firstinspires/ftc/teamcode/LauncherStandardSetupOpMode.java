package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Launcher Test", group = "Robot")
public class LauncherStandardSetupOpMode extends StandardSetupOpMode {

    @Override
    public void init() {
        // 1. Get all hubs (Control Hub + Expansion Hub)
        allHubs = hardwareMap.getAll(LynxModule.class);

        // 2. Set to AUTO mode for multi-threading
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Parts
        launcher = new Launcher(this);
        partsList.add(Map.entry(launcher, 20));

        // Init parts
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            entry.getKey().init();
        }

        // Setup the thread pool
        threadPool = Executors.newScheduledThreadPool(partsList.size());
    }
}
