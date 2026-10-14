package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class Eye extends RobotPart {
    //private static final double CLOSE_ENOUGH_DEGREES = 6.0;
    private final Limelight3A limelight;
    private final Mode mode;
    private LLResult resultInUse;
    private final StandardSetupOpMode ssom;
    private final List<Integer> redFiducials = Arrays.asList(30, 31, 32, 33, 34, 35, 36, 37);
    private final List<Integer> blueFiducials = Arrays.asList(38, 39, 40, 41, 42, 43, 44, 45);
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
        this.mode = Mode.NONE;
        // Setup limelight
        limelight = ssom.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(150); // This sets how often we ask Limelight for data (100 times per second)

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
    }

    @Override
    public void loop() {
        // Sanity
        if (limelight == null) {
            ssom.telemetry.addLine("No Limelight!");
            ssom.telemetry.update();
            return;
        }

        if (mode == Mode.LOCATE_POLLEN) {
            resultInUse = limelight.getLatestResult();
            if (resultInUse != null) {
                List<LLResultTypes.FiducialResult> fiducials = resultInUse.getFiducialResults();
                if (fiducials != null) {
                    for(LLResultTypes.FiducialResult fiducial : fiducials) {
                        // See which fiducial it is

                        if (ssom.color == StandardSetupOpMode.COLOR.BLUE && blueFiducials.contains(fiducial.getFiducialId())) {
                            // we're in Blue mode and see a blue fiducial
                        } else if (ssom.color == StandardSetupOpMode.COLOR.RED && redFiducials.contains(fiducial.getFiducialId())) {
                            // we're in Red mode and see a red fiducial
                        }
                    }
                }
            }
        }
        if (mode == Mode.FIND_HIVE) {

        }
        // Cleanup
        //limelight.stop();
    }

    @Override
    public void start() {
        // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }
}
