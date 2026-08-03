package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Robot Setup Super Class", group = "Robot")
@Disabled
public class StandardSetupOpMode extends OpMode {

    // Game Definitions
    public enum COLOR {
        RED,
        BLUE
    }
    public enum POSITION {
        CLOSE,
        FAR
    }
    public static String colorToString(COLOR color)
    {
        return (color == COLOR.RED) ? "Red" : "Blue";
    }
    public static String positionToString(POSITION pos)
    {
        return (pos == POSITION.CLOSE) ? "Close" : "Far";
    }

    // Variables
    public boolean ignoreGamepad = false;
    public COLOR color = COLOR.BLUE;
    public POSITION position = POSITION.CLOSE;

    // Robot parts
    protected final ArrayList<Map.Entry<RobotPart, Integer> > partsList = new ArrayList<>();
    public Motion motion;
    //public BallLifter ballLifter;

    private ScheduledExecutorService threadPool;

    protected List<LynxModule> allHubs;

    @Override
    public void init() {
        // 1. Get all hubs (Control Hub + Expansion Hub)
        allHubs = hardwareMap.getAll(LynxModule.class);

        // 2. Set to AUTO mode for multi-threading
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Parts
        motion = new Motion(this);
        //ballLifter = new BallLifter(this);

        // Add parts to parts list
        partsList.add(Map.entry(motion, 20));
        //partsList.add(Map.entry(ballLifter, 20));

        // Init parts
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            entry.getKey().init();
        }

        // Setup the thread pool
        threadPool = Executors.newScheduledThreadPool(partsList.size());
    }

    @Override
    public void start(){
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

    @Override
    public void stop(){
        // Stop accepting new tasks
        threadPool.shutdown();
        try {
            // 2. Wait a reasonable time for existing tasks to terminate
            if (!threadPool.awaitTermination(500, TimeUnit.MILLISECONDS)) {
                // 3. Cancel currently executing tasks if they take too long
                threadPool.shutdownNow();
                // 4. Wait a bit longer for tasks to respond to being cancelled
                if (!threadPool.awaitTermination(500, TimeUnit.MILLISECONDS)) {
                    telemetry.addLine("Pool did not terminate!");
                    telemetry.update();
                }
            }
        } catch (InterruptedException ie) {
            // (Re-)Cancel if current thread also interrupted
            threadPool.shutdownNow();
            // Preserve interrupt status
            Thread.currentThread().interrupt();
        }

        // Stop the robot parts
        motion.stop();
    }

    /**
     * Method sets up this specific opmode
     * @param color color robot should use
     * @param position position robot is starting on
     * @param ignoreGamepad true to ignore gamepad input
     */
    protected void setup(COLOR color, POSITION position, boolean ignoreGamepad) {
        // Setup position and color for this opmode
        this.color = color;
        this.position = position;

        // Should we ignore the gamepad or not?
        this.ignoreGamepad = ignoreGamepad;
    }
}
