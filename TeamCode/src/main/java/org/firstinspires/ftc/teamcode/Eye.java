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
    private static final double FRACTIONAL_CORRECTION = 0.4; // Percentage of correction to apply

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
                                }
                            }
                            if (foundHiveState && foundAimPoint)
                                break;
                        }
                    }
                }
            }
            activeTarget = aimResult != null;

            // TODO - If we didn't find an aim point, scan search
            if (!foundAimPoint) {

            }

            // Here we update the camera position
            // TODO - Handle wrap around
            targetCamAngleDeg = currentCamAngleDeg + (FRACTIONAL_CORRECTION * degreeToTurn);
            lastDegreeToTurn = degreeToTurn;
            if(!ssom.ignoreGamepad) {
                if (Math.abs(ssom.gamepad2.left_stick_y) > 0.05 || Math.abs(ssom.gamepad2.left_stick_x) >= 0.05) {
                    targetCamAngleDeg = Math.toDegrees(Math.atan2(-ssom.gamepad2.left_stick_y, -ssom.gamepad2.left_stick_x)) - 90.0;
                    if (targetCamAngleDeg < -180) targetCamAngleDeg += 360.0;
                    if (targetCamAngleDeg > 180) targetCamAngleDeg -= 360.0;
                    degreeToTurn = CLOSE_ENOUGH_DEGREES + 1.0;
                }
            }
            lastCamMoved = Math.abs(degreeToTurn) > CLOSE_ENOUGH_DEGREES;
            if (Math.abs(degreeToTurn) > CLOSE_ENOUGH_DEGREES) {
                camServo.setPosition(calcPowerFromAngle(targetCamAngleDeg));
            }
        }
        else if (mode == Mode.LOCATE_POLLEN) {

        }
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
