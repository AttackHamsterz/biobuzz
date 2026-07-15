package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
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
    public BallLifter ballLifter;
    public PodTest podTest;

    public Motion motion;

    public GamepadBuffer gamepadBuffer;

    private ScheduledExecutorService threadPool;

    @Override
    public void init() {
        // Parts
        ballLifter = new BallLifter(this);
        podTest = new PodTest(this);

        // Add parts to parts list
        //partsList.add(Map.entry(ballLifter, 20));
        partsList.add(Map.entry(podTest, 10));

        // Setup the thread pool
        threadPool = Executors.newScheduledThreadPool(2);
    }

    @Override
    public void start(){
        // Launch the robot parts
        for (Map.Entry<RobotPart, Integer> entry : partsList) {
            threadPool.scheduleWithFixedDelay(entry.getKey()::loop, 0, entry.getValue(), TimeUnit.MILLISECONDS);
        }
    }


    @Override
    public void loop() {
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
};
