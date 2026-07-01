package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotPart extends Thread {
    // Opmode reference
    //protected StandardSetupOpMode ssom;
    protected boolean running;

    // DEBUG drives telemetry output
    // Set bit positions to enable disable outputs
    //     1 - Motion
    //     2 - Lifter
    //     4 - Eye
    //    8 - Intake
    //   16 - Elevator
    //   So to dump Motion, Eye and Intake: 0b00001101 -or- 1+4+8
    public static final int DEBUG = 13;
    protected static final long LOOP_PAUSE_MS = 100;

    /**
     * Force implementing classes to implement the run class.
     * This implements the things this body part can do in parallel with
     * other body parts.
     */
    @Override
    public abstract void run();

    /**
     * Display relevant telemetry to the user for debugging
     *
     * @param telemetry place to put the info
     */
    public abstract void getTelemetry(Telemetry telemetry);

    /**
     * Tell the robot part we are running threads
     */
    public void setRunning() {
        running = true;
    }

    /**
     * Signal any robot part threads to stop and cleanup
     */
    public void safeStop() {
        running = false;
    }

    /**
     * Traditional sleep for a robot part (to avoid saturating threads)
     */
    public void sleep() {
        try {
            sleep(LOOP_PAUSE_MS);
        } catch (InterruptedException ignored) {
        }
    }
}

