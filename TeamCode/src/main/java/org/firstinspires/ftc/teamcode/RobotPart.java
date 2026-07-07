package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotPart {
    // Opmode reference
    protected StandardSetupOpMode ssom;
    protected boolean running;

    // DEBUG drives telemetry output
    // Set bit positions to enable disable outputs
    //    1 - Motion
    //    2 - BallLifter
    //    4 - Eye
    //    8 - Intake
    //   16 - Elevator
    //   So to dump Motion, Eye and Intake: 0b00001101 -or- 1+4+8
    public static final int DEBUG = 13;

    /**
     * Force implementing classes to implement the loop method.
     * This implements the things this robot part can do in parallel with
     * other robot parts.  The scheduler wil handle how often this method is called.
     */
    public abstract void loop();

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
}

