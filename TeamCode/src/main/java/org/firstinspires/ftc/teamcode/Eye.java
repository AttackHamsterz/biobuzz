package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class Eye extends RobotPart {
    //private static final double CLOSE_ENOUGH_DEGREES = 6.0;
    private final Servo camServo;
    private final Limelight3A limelight;
    private final Mode mode;
    private LLResult resultInUse;
    private final StandardSetupOpMode ssom;
    private final List<Integer> redFiducials = Arrays.asList(30, 31, 32, 33, 34, 35, 36, 37);
    private final List<Integer> blueFiducials = Arrays.asList(38, 39, 40, 41, 42, 43, 44, 45);
    private final List<Integer> centerFiducials = Arrays.asList(39, 40, 43, 44 ,35, 36, 32, 31);
    private List<Integer> flowerBlueFiducials = Arrays.asList(38,39,40,41);
    private List<Integer> nonFlowerBlueFiducials = Arrays.asList(42,43,44,45);
    private List<Integer> nonFlowerRedFiducials = Arrays.asList(34,35,36,37);
    private List<Integer> flowerRedFiducials = Arrays.asList(33,32,31,30);
    public List<Hive> hives;
    private Fiducial targetedFiducial;
    private double degreeToTurn = 90;
    private double positionBeingSet = 0;
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

        // Setup limelight
        limelight = ssom.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data
        this.hives = new ArrayList<Hive>();
        this.hives.add(new Hive("blue", "blueFlower", flowerBlueFiducials));
        this.hives.add(new Hive("blue", "blueNonFlower", nonFlowerBlueFiducials));
        this.hives.add(new Hive("red", "redNonFlower", nonFlowerRedFiducials));
        this.hives.add(new Hive("red", "redFlower", flowerRedFiducials));

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
    }

    @Override
    public void start() {
        // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
        limelight.start();
        resetCamera();
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
            resultInUse = limelight.getLatestResult();

            if (resultInUse != null) {
                Pose3D botPose = resultInUse.getBotpose();
                YawPitchRollAngles llOrientation = botPose.getOrientation();
                Position llPosition = botPose.getPosition();

                List<LLResultTypes.FiducialResult> fiducials = resultInUse.getFiducialResults();
                if (fiducials != null) {
                    for(LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId();
                        Position fiducialResult = fiducial.getTargetPoseRobotSpace().getPosition();

                        for (Hive hive : this.hives) {
                            for (Fiducial hiveFiducial : hive.fiducials)
                            {
                                if (hiveFiducial.id == id) {
                                    hiveFiducial.x = fiducialResult.x;
                                    hiveFiducial.y = fiducialResult.y;
                                    hiveFiducial.z = fiducialResult.z;

                                    if (hiveFiducial.z >= 0.85) {
                                        hive.isUp = true;
                                    } else {
                                        hive.isUp = false;
                                    }
                                }
                            }
                        }
                        // See which fiducial it is
                        if (ssom.color == StandardSetupOpMode.COLOR.BLUE && blueFiducials.contains(fiducial.getFiducialId())) {
                            // we're in Blue mode and see a blue fiducial
                            Hive hive = getActiveHive("blue");
                            targetedFiducial = getTargetFiducial(hive);
                            degreeToTurn = Math.toDegrees(Math.atan2(targetedFiducial.y, targetedFiducial.x));
                            positionBeingSet = calcPositionFromAngle(degreeToTurn);
                            //camServo.setPosition(positionBeingSet);
                        } else if (ssom.color == StandardSetupOpMode.COLOR.RED && redFiducials.contains(fiducial.getFiducialId())) {
                            // we're in Red mode and see a red fiducial
                            Hive hive = getActiveHive("red");
                            targetedFiducial = getTargetFiducial(hive);
                            degreeToTurn = Math.toDegrees(Math.atan2(targetedFiducial.y, targetedFiducial.x));
                            positionBeingSet = calcPositionFromAngle(degreeToTurn);
                            //camServo.setPosition(positionBeingSet);
                        } else{
                            resetCamera();
                            //start searching
                        }
                    }
                }
            }
        }
        else if (mode == Mode.LOCATE_POLLEN) {

        }
        //Cleanup
        //limelight.stop();
    }

    public boolean nonFlowerUp(){
        if (ssom.color == StandardSetupOpMode.COLOR.BLUE) {
            for (Hive hive : this.hives) {
                if (hive.name.equals("blueNonFlower")) {
                    return hive.isUp;
                }
            }
        }
        else{
            for (Hive hive : this.hives) {
                if (hive.name.equals("redNonFlower")) {
                    return hive.isUp;
                }
            }
        }
        return false;
    }

    public boolean flowerup(){
        if(ssom.color == StandardSetupOpMode.COLOR.BLUE ) {
            for (Hive hive : this.hives) {
                if (hive.name.equals("blueFlower")) {
                    return hive.isUp;
                }
            }
        }
        else{
            for (Hive hive : this.hives) {
                if (hive.name.equals("redFlower")) {
                    return hive.isUp;
                }
            }
        }
        return false;
    }

    public Hive getActiveHive(String color){
        for (Hive hive : this.hives) {
            if(hive.color.equals(color) && hive.isUp){
                return hive;
            }
        }
        return null;
    }

    public Fiducial getTargetFiducial(Hive hive){
        Fiducial closest = null;
        for (Fiducial fiducial : hive.fiducials){
            if(centerFiducials.contains(fiducial.id)) {
                if (closest == null) {
                    closest = fiducial;
                } else {
                    Double distance = Math.abs(fiducial.x) + Math.abs(fiducial.y) + Math.abs(fiducial.z);

                    if (distance < (Math.abs(closest.x) + Math.abs(closest.y) + Math.abs(closest.z)))
                        closest = fiducial;
                }
            }
        }

        return closest;
    }

    public double calcPositionFromAngle(double angle){
        return (angle - 90) * (2.0/350);
    }

    public void resetCamera(){
        camServo.setPosition(0.0);
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 4) != 0) {
            telemetry.addData("Angle",degreeToTurn - 90);
            telemetry.addData("Position",positionBeingSet);
            telemetry.addData("Mode", this.mode);
            telemetry.addData("llresult", resultInUse.getTimestamp());
            if (targetedFiducial != null) {
                telemetry.addData("Targeted Fiducial", targetedFiducial.id);
                telemetry.addData(" x:", targetedFiducial.x );
                telemetry.addData(" y:", targetedFiducial.y );
                telemetry.addData(" z:", targetedFiducial.z );
            }

            for (Hive hive : this.hives) {
                telemetry.addData(hive.name + " isUp: ", hive.isUp);

                for (Fiducial fiducial : hive.fiducials) {
                    //telemetry.addData("fiducial: ", fiducial.id);
                    telemetry.addData("  " + fiducial.id + " x:", fiducial.x );
                    telemetry.addData("  " + fiducial.id + " y:", fiducial.y );
                    telemetry.addData("  " + fiducial.id + " z:", fiducial.z );
                }
            }
        }
    }
}
