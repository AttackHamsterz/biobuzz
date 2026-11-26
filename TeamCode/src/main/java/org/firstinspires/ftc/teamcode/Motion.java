package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.PathSegment;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Motion extends RobotPart {
    public Follower follower;
    private double externalTurn;
    private double f, s, t;

    private static final double DEAD_ZONE_SIZE = 0.05;
    private static final double EXPONENT = 3.0;

    private boolean turtleMode = false;

    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        follower = Constants.create(ssom.hardwareMap);
        externalTurn = 0;
    }
    public void setTurn(double turn) { externalTurn = turn; }

    @Override
    public void init() {
    }

    @Override
    public void start(){
        if (!ssom.ignoreGamepad) {
            follower.setPose(new Pose(0, 0, 0));
            follower.manual(0, 0, 0);
            follower.update();
        }
    }

    /**
     * Remaps input above dead zone to [0, 1] range and applies a power curve.
     */
    private double conditionInput(double rawInput) {
        if (Math.abs(rawInput) < DEAD_ZONE_SIZE) {
            return 0.0;
        }

        // Scale [DEAD_ZONE_SIZE, 1.0] -> [0.0, 1.0]
        double remapped = (Math.abs(rawInput) - DEAD_ZONE_SIZE) / (1.0 - DEAD_ZONE_SIZE);

        // Preserve sign and apply polynomial power curve
        return Math.signum(rawInput) * Math.pow(remapped, EXPONENT);
    }

    @Override
    public void loop() {
        if(!ssom.ignoreGamepad) {
            // In and out of turtle mode
            if (ssom.gamepad1.touchpadWasPressed()) {
                turtleMode = !turtleMode;
            }

            // Movement from joysticks
            double rawForward = -ssom.gamepad1.left_stick_y;    // Up on stick is negative but we want the robot to go forward (positive y)
            double rawLateral = -ssom.gamepad1.left_stick_x;     // Left on stick is negative, want positive to match autonomous follower
            double rawHeading = -ssom.gamepad1.right_stick_x;   // Left on stick is negative but we want a positive CCW rotation)

            // Scale applied to driving (turtle mode of course but also slow down when intake is humming)
            double TURTLE_POWER_SCALE = 0.6;        // Turtle mode max speed
            double INTAKE_PENALTY_SCALE = 0.4;      // Percent speed to remove when fully intaking
            double INTAKE_PENALTY_THRESHOLD = 0.7;  // When intakes are running over this percent, slow down
            double scale = 1.0;
            if (turtleMode){
                scale = TURTLE_POWER_SCALE;
            }
            else if(ssom.gamepad1.left_trigger > INTAKE_PENALTY_THRESHOLD || ssom.gamepad1.right_trigger > INTAKE_PENALTY_THRESHOLD){
                double penaltyPercent = Range.clip((Math.max(ssom.gamepad1.left_trigger, ssom.gamepad1.right_trigger) - INTAKE_PENALTY_THRESHOLD) / (1.0 - INTAKE_PENALTY_THRESHOLD),0, 1);
                scale = 1.0 - penaltyPercent * INTAKE_PENALTY_SCALE;
            }

            // Conditioning and application
            f = conditionInput(rawForward * scale);
            s = conditionInput(rawLateral * scale);
            t = conditionInput(rawHeading * scale);
            boolean inDeadzone = (f == 0.0 && s == 0.0 && t == 0.0);
            if (inDeadzone)
                follower.manual(0, 0, 0);
            else
                follower.manual(f, s, t);
            follower.update();
        }
    }

    public void stop() {
        follower.manual(0,0,0);
        follower.update();
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 1) !=0) {
            Pose pose = follower.localizer.pose();
            PathSegment segment = follower.currentSegment();
            Pose targetPose = (segment == null) ? null : follower.currentSegment().endPose();

            if(pose != null) {
                telemetry.addData("f", f);
                telemetry.addData("s", s);
                telemetry.addData("t", t);
                if(pose != null) {
                    telemetry.addData("X", pose.x());
                    telemetry.addData("Y", pose.y());
                    telemetry.addData("H", Math.toDegrees(pose.heading()));
                }
                if(targetPose != null) {
                    telemetry.addData("TX", targetPose.x());
                    telemetry.addData("TY", targetPose.y());
                    telemetry.addData("TH", Math.toDegrees(targetPose.heading()));
                }
            }
        }
    }
}
