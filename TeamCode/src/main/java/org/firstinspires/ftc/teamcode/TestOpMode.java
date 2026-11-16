package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Map;
import java.util.concurrent.Executors;


@TeleOp(name="TeleOp: test op mode", group="Robot")
public class TestOpMode extends StandardSetupOpMode{

    @Override
    public void init() {
        // 1. Get all hubs (Control Hub + Expansion Hub)
        allHubs = hardwareMap.getAll(LynxModule.class);

        // 2. Set to AUTO mode for multi-threading
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        //This is how you set up your testing part
        //eye = new Eye(this);
        //partsList.add(Map.entry(eye, 20));

        // Init parts
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            entry.getKey().init();
        }
        // Setup the thread pool
        threadPool = Executors.newScheduledThreadPool(partsList.size());
    }

    @Override
    public void loop() {
        // Ensure ssom loop is called
        super.loop();

        // Update telemetry
        if(RobotPart.DEBUG != 0) {
            super.partsList.forEach(part -> part.getKey().getTelemetry(telemetry));
            if (!super.partsList.isEmpty()) telemetry.update();
        }
    }
}
