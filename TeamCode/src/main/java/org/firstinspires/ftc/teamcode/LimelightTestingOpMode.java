package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Lime Light Test", group = "Robot")
@Disabled
public class LimelightTestingOpMode extends StandardSetupOpMode {

    private Eye eye;

    @Override
    public void init() {
        // 1. Get all hubs (Control Hub + Expansion Hub)
        allHubs = hardwareMap.getAll(LynxModule.class);

        // 2. Set to AUTO mode for multi-threading
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        eye= new Eye(this);
        partsList.add(Map.entry(eye, 20));

        // Init parts
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            entry.getKey().init();
        }

        // Setup the thread pool
        threadPool = Executors.newScheduledThreadPool(partsList.size());
    }

    @Override public void loop() {
        // Ensure ssom loop is called
        super.loop();

        // Update telemetry
        if(RobotPart.DEBUG != 0) {
            super.partsList.forEach(part -> part.getKey().getTelemetry(telemetry));
            if (!super.partsList.isEmpty()) telemetry.update();
        }
    }
}