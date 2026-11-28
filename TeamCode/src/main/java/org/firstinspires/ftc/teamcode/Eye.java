package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

public class Eye extends RobotPart {
    private static final double HIVE_UP_HEIGHT = 0.85;
    private static final double CLOSE_ENOUGH_DEGREES = 2.0; // Seems close enough
    // Fraction of the measured error to correct per fresh measurement. The correction is now
    // applied to where the camera pointed when the frame was captured (not the latest command),
    // so 1.0 means "aim straight at it". Was 0.4 on the old add-to-last-command scheme.
    private static final double FRACTIONAL_CORRECTION = 1.0;
    // Frame age covers capture -> Control Hub. This adds the camera servo's own travel delay.
    // 2026-11-27 log: commands showed up in the camera's error ~110-150 ms later, frames ~108 ms old.
    private static final double CAM_SERVO_LAG_MS = 25.0;
    private static final double CAM_LIMIT_DEG = 175.0; // servo travel each way (see calcPowerFromAngle)

    private final Servo camServo;
    private final Limelight3A limelight;
    private final Mode mode;

    private Position hiveResult;
    private Position aimResult;

    private boolean activeTarget = false;
    private boolean flowerHiveUp = false; // Game starts with non flower hive up
    private double currentCamAngleDeg;
    private double targetCamAngleDeg;

    // Logging only (read by LaunchLog from the launcher thread)
    private volatile double lastDegreeToTurn;
    private volatile boolean lastCamMoved;
    private volatile boolean lastFrameNew;
    private volatile double lastFrameAgeMs = Double.NaN;
    private volatile int frameCount;
    private double prevFrameTs = -1;
    private volatile boolean lastFresh;
    private volatile int lastAimId = -1;
    private volatile int lastAimTags;
    private volatile int lastCenterSeen;
    private volatile double lastCamAtCaptureDeg = Double.NaN;

    // Last aim measurement acted on; Limelight often republishes the same one
    private double prevAimX = Double.NaN;
    private double prevAimY = Double.NaN;

    // Recent camera commands (time, angle), to look up where the camera pointed at frame capture
    private static final int CMD_HIST = 32;
    private final long[] cmdTimeMs = new long[CMD_HIST];
    private final double[] cmdAngleDeg = new double[CMD_HIST];
    private int cmdCount = 0;
    public enum Mode {
        LOCATE_POLLEN(0),    // Locating the pollen
        FIND_HIVE(1),       // Finding the fiducials
        NONE(3);            // No mode, do nothing

        private final int value;
        Mode(int value){
            this.value = value;
        }
    }

    @NonNull
    public String toString()
    {
        String output;
        int value = 0;
        switch(value){
            case 0:
                output = "Locate Pollen Mode";
                break;
            case 1:
                output = "Find Hive Mode";
                break;
            case 2:
            default:
                output = "No Mode";
                break;
        };
        return output;
    }

    public Eye(StandardSetupOpMode ssom) {
        this.ssom = ssom;
        this.mode = Mode.FIND_HIVE;

        // Setup camera servo
        camServo = ssom.hardwareMap.get(Servo.class, "camServo");
        camServo.setDirection(Servo.Direction.FORWARD);

        // Setup limelight
        limelight = ssom.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data

            /* Table of distances for in/s to angle degrees
            deltaAim = new ArrayList<>(List.of(
                    new VelocityAngleEntry(-50.0, 50.0),
                    new VelocityAngleEntry(-15.0, 15.0),
                    new VelocityAngleEntry(-5.0, 10.0),
                    new VelocityAngleEntry(0, 0),
                    new VelocityAngleEntry(5.0, -10.0),
                    new VelocityAngleEntry(15.0, -15.0),
                    new VelocityAngleEntry(50.0, -45.0)
            ));
            */
    }

    @Override
    public void init() {
        camServo.setPosition(0.5);
        currentCamAngleDeg = 0;
        targetCamAngleDeg = 0;
    }

    @Override
    public void start() {
        // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        // Sanity
        if (limelight == null) {
            ssom.telemetry.addLine("No Limelight!");
            ssom.telemetry.update();
            return;
        }
        if (mode == Mode.FIND_HIVE) {
            // Get results and latest servo position closest to results fetch
            LLResult resultInUse = limelight.getLatestResult();
            currentCamAngleDeg = calcAngleFromPower(camServo.getPosition());
            double degreeToTurn = 0;

            // Logging only: is this a new frame, and how old is it
            if (resultInUse != null) {
                double ts = resultInUse.getTimestamp();
                lastFrameNew = ts != prevFrameTs;
                if (lastFrameNew) frameCount++;
                prevFrameTs = ts;
                lastFrameAgeMs = resultInUse.getStaleness()
                        + resultInUse.getCaptureLatency() + resultInUse.getTargetingLatency();
            } else {
                lastFrameNew = false;
                lastFrameAgeMs = Double.NaN;
            }

            boolean foundHiveState = false;
            boolean foundAimPoint = false;
            int aimId = -1;       // first center tag found
            int aimTags = 1;      // 2 when its partner was averaged in
            int centerSeen = 0;   // logging only
            hiveResult = null;
            aimResult = null;

            if (resultInUse != null) {
                // We are looking for two things, which hive is up and aim point
                // Once we have that we stop looking

                List<LLResultTypes.FiducialResult> fiducials = resultInUse.getFiducialResults();
                if (fiducials != null) {
                    if (ssom.color == StandardSetupOpMode.COLOR.BLUE) {
                        // Pipeline should sort them closest to furthest
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId();
                            if (!foundHiveState) {
                                // flowerBlueFiducials = Arrays.asList(38,39,40,41);
                                if (id >= 38 && id <= 41) {
                                    hiveResult = fiducial.getTargetPoseRobotSpace().getPosition();
                                    if (hiveResult.z >= HIVE_UP_HEIGHT) {
                                        flowerHiveUp = true;
                                    } else {
                                        flowerHiveUp = false;
                                    }
                                    foundHiveState = true;
                                }
                                // nonFlowerBlueFiducials = Arrays.asList(42,43,44,45);
                                else if (id >= 42 && id <= 45) {
                                    hiveResult = fiducial.getTargetPoseRobotSpace().getPosition();
                                    if (hiveResult.z >= HIVE_UP_HEIGHT) {
                                        flowerHiveUp = false;
                                    } else {
                                        flowerHiveUp = true;
                                    }
                                    foundHiveState = true;
                                }
                            }
                            if (!foundAimPoint) {
                                // centerBlueFiducials = Arrays.asList(39, 40, 43, 44);
                                if (id == 39 || id == 40 || id == 43 || id == 44) {
                                    aimResult = fiducial.getTargetPoseRobotSpace().getPosition();
                                    degreeToTurn = -Math.toDegrees(Math.atan2(aimResult.y, aimResult.x)) + 90.0;
                                    if (degreeToTurn > 180.0)
                                        degreeToTurn -= 360.0;
                                    else if (degreeToTurn < -180.0)
                                        degreeToTurn += 360.0;
                                    foundAimPoint = true;
                                    aimId = id;
                                }
                            }
                            if (foundHiveState && foundAimPoint)
                                break;
                        }
                    } else {
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            int id = fiducial.getFiducialId();
                            if (!foundHiveState) {
                                // flowerRedFiducials = Arrays.asList(33,32,31,30);
                                if (id >= 30 && id <= 33) {
                                    hiveResult = fiducial.getTargetPoseRobotSpace().getPosition();
                                    if (hiveResult.z >= HIVE_UP_HEIGHT) {
                                        flowerHiveUp = true;
                                    } else {
                                        flowerHiveUp = false;
                                    }
                                    foundHiveState = true;
                                }
                                // nonFlowerRedFiducials = Arrays.asList(34,35,36,37);
                                else if (id >= 34 && id <= 37) {
                                    hiveResult = fiducial.getTargetPoseRobotSpace().getPosition();
                                    if (hiveResult.z >= HIVE_UP_HEIGHT) {
                                        flowerHiveUp = false;
                                    } else {
                                        flowerHiveUp = true;
                                    }
                                    foundHiveState = true;
                                }
                            }
                            if (!foundAimPoint) {
                                // centerRedFiducials = Arrays.asList(35, 36, 32, 31);
                                if (id == 31 || id == 32 || id == 35 || id == 36) {
                                    aimResult = fiducial.getTargetPoseRobotSpace().getPosition();
                                    degreeToTurn = -Math.toDegrees(Math.atan2(aimResult.y, aimResult.x)) + 90.0;
                                    if (degreeToTurn > 180.0)
                                        degreeToTurn -= 360.0;
                                    else if (degreeToTurn < -180.0)
                                        degreeToTurn += 360.0;
                                    foundAimPoint = true;
                                    aimId = id;
                                }
                            }
                            if (foundHiveState && foundAimPoint)
                                break;
                        }
                    }

                    // Aim at the midpoint of the cell's two center tags when both are visible.
                    // Limelight lists tags largest first and the center tags are almost the same
                    // size, so "first center tag" switched between them frame to frame (~12 cm).
                    if (foundAimPoint) {
                        int partner = centerPartner(aimId);
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            if (fiducial.getFiducialId() == partner) {
                                Position p = fiducial.getTargetPoseRobotSpace().getPosition();
                                aimResult = new Position(p.unit,
                                        (aimResult.x + p.x) / 2.0,
                                        (aimResult.y + p.y) / 2.0,
                                        (aimResult.z + p.z) / 2.0,
                                        p.acquisitionTime);
                                aimTags = 2;
                                break;
                            }
                        }
                        degreeToTurn = -Math.toDegrees(Math.atan2(aimResult.y, aimResult.x)) + 90.0;
                        if (degreeToTurn > 180.0)
                            degreeToTurn -= 360.0;
                        else if (degreeToTurn < -180.0)
                            degreeToTurn += 360.0;
                    }
                    // Logging only: how many of our center tags are in view
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (isCenterTag(fiducial.getFiducialId())) centerSeen++;
                    }
                }
            }
            activeTarget = aimResult != null;
            lastAimId = foundAimPoint ? aimId : -1;
            lastAimTags = foundAimPoint ? aimTags : 0;
            lastCenterSeen = centerSeen;

            // TODO - If we didn't find an aim point, scan search
            if (!foundAimPoint) {

            }

            // Here we update the camera position
            // TODO - Handle wrap around
            // Only act on a fresh measurement: a new frame whose aim point changed. The frame is
            // ~100 ms old, so the error is measured from where the camera pointed back then, not
            // from the latest command. Correcting from the latest command re-applied the same
            // error every loop until the camera's own move showed up, and it overshot.
            long nowMs = System.currentTimeMillis();
            boolean moveCam = false;
            lastDegreeToTurn = degreeToTurn;
            lastFresh = false;
            if (foundAimPoint && lastFrameNew
                    && (aimResult.x != prevAimX || aimResult.y != prevAimY)) {
                prevAimX = aimResult.x;
                prevAimY = aimResult.y;
                lastFresh = true;
                double camAtCapture = commandedAngleAt(
                        nowMs - (long) (lastFrameAgeMs + CAM_SERVO_LAG_MS));
                lastCamAtCaptureDeg = camAtCapture;
                double desired = Math.max(-CAM_LIMIT_DEG, Math.min(CAM_LIMIT_DEG,
                        camAtCapture + FRACTIONAL_CORRECTION * degreeToTurn));
                // Inside the band, hold the target so neither camera nor turret chase frame noise
                if (Math.abs(desired - targetCamAngleDeg) > CLOSE_ENOUGH_DEGREES) {
                    targetCamAngleDeg = desired;
                    moveCam = true;
                }
            }
            if(!ssom.ignoreGamepad) {
                if (Math.abs(ssom.gamepad2.left_stick_y) > 0.05 || Math.abs(ssom.gamepad2.left_stick_x) >= 0.05) {
                    targetCamAngleDeg = Math.toDegrees(Math.atan2(-ssom.gamepad2.left_stick_y, -ssom.gamepad2.left_stick_x)) - 90.0;
                    if (targetCamAngleDeg < -180) targetCamAngleDeg += 360.0;
                    if (targetCamAngleDeg > 180) targetCamAngleDeg -= 360.0;
                    moveCam = true;
                }
            }
            lastCamMoved = moveCam;
            if (moveCam) {
                camServo.setPosition(calcPowerFromAngle(targetCamAngleDeg));
                recordCommand(nowMs, targetCamAngleDeg);
            }
        }
        else if (mode == Mode.LOCATE_POLLEN) {

        }
    }

    // The two center tags of each cell's 4-tag cluster (blue 38-41 / 42-45, red 30-33 / 34-37)
    private static int centerPartner(int id) {
        switch (id) {
            case 39: return 40;
            case 40: return 39;
            case 43: return 44;
            case 44: return 43;
            case 31: return 32;
            case 32: return 31;
            case 35: return 36;
            case 36: return 35;
            default: return -1;
        }
    }

    private boolean isCenterTag(int id) {
        if (ssom.color == StandardSetupOpMode.COLOR.BLUE)
            return id == 39 || id == 40 || id == 43 || id == 44;
        return id == 31 || id == 32 || id == 35 || id == 36;
    }

    private void recordCommand(long timeMs, double angleDeg) {
        int i = cmdCount % CMD_HIST;
        cmdTimeMs[i] = timeMs;
        cmdAngleDeg[i] = angleDeg;
        cmdCount++;
    }

    /** The camera angle most recently commanded at or before timeMs. */
    private double commandedAngleAt(long timeMs) {
        if (cmdCount == 0) return currentCamAngleDeg;
        int n = Math.min(cmdCount, CMD_HIST);
        for (int k = 1; k <= n; k++) {
            int i = (cmdCount - k) % CMD_HIST;
            if (cmdTimeMs[i] <= timeMs) return cmdAngleDeg[i];
        }
        return cmdAngleDeg[(cmdCount - n) % CMD_HIST]; // older than history: oldest known
    }

    public boolean nonFlowerUp(){
        return !flowerHiveUp;
    }

    public boolean flowerup(){
        return flowerHiveUp;
    }

    public boolean activeTarget(){
        return activeTarget;
    }

    public double getTargetAngleDeg(){
        return targetCamAngleDeg;
    }

    // ---- Logging getters (LaunchLog) ----
    public double getCamAngleDeg() { return currentCamAngleDeg; }
    public double getLastDegreeToTurn() { return lastDegreeToTurn; }
    public boolean getLastCamMoved() { return lastCamMoved; }
    public boolean getLastFrameNew() { return lastFrameNew; }
    public double getLastFrameAgeMs() { return lastFrameAgeMs; }
    public int getFrameCount() { return frameCount; }
    public Position getAimResult() { return aimResult; }
    public boolean getLastFresh() { return lastFresh; }
    public double getLastCamAtCaptureDeg() { return lastCamAtCaptureDeg; }
    public int getLastAimId() { return lastAimId; }
    public int getLastAimTags() { return lastAimTags; }
    public int getLastCenterSeen() { return lastCenterSeen; }

    /**
     * Given an angle in degrees we calculate the power that's applied to the camera servo.
     * 0.0 rotates 175 degrees counter clockwise, 0.5 is 0 degrees (straight ahead), 1.0 rotates
     * 175 degrees clockwise
     * @param angle degrees to rotate to
     * @return (power from 0.0 to 1.0)
     */
    public double calcPowerFromAngle(double angle){
        return 0.5 + angle * 0.00285714285; // (1/350 power/degree)
    }

    public double calcAngleFromPower(double power){
        return (power - 0.5) * 350.0;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 4) != 0) {
            telemetry.addData("Mode", this.mode);
            telemetry.addData("CamAngle",currentCamAngleDeg);
            telemetry.addData("TargetAngle",targetCamAngleDeg);
            telemetry.addData("FlowerHiveUp", flowerHiveUp);
            Position temp = aimResult;
            if(temp != null){
                telemetry.addData("aim x", temp.x);
                telemetry.addData("aim y", temp.y);
                telemetry.addData("aim z", temp.z);
            }

        }
    }
}
