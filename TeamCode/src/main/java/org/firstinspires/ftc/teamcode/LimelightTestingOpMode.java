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

public class LimelightTestingOpMode extends StandardSetupOpMode {


    protected List<LynxModule> allHubs;
    protected final ArrayList<Map.Entry<RobotPart, Integer>> partsList = new ArrayList<>();
    private ScheduledExecutorService threadPool;
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

    }

    @Override
    public void start() {
        // Start parts
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            entry.getKey().start();
        }

        // Launch the robot part loops
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            threadPool.scheduleWithFixedDelay(entry.getKey()::loop, 0, entry.getValue(), TimeUnit.MILLISECONDS);
        }
    }

    @Override
    public void loop() {
        // Clear cache on each loop, speeds up ic2/analog/digital device queries
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}