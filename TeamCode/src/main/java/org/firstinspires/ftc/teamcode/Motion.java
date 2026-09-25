package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Motion extends RobotPart {
    public Follower follower;
    private double externalTurn;
    private double f, s, t;

    private static final double DEAD_ZONE_SIZE = 0.05;
    private static final double EXPONENT = 3.0;

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
        follower.setPose(new Pose(0,0,0));
        follower.manual(0, 0, 0);
        follower.update();
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
            if (ssom.gamepad1.start)
            {
                follower.manual(0,0,0);
                return;
            }
            double rawForward = -ssom.gamepad1.left_stick_y;
            double rawLateral = -ssom.gamepad1.left_stick_x;
            double rawHeading = -ssom.gamepad1.right_stick_x;
            double forward = conditionInput(rawForward);
            double lateral = conditionInput(rawLateral);
            double heading = conditionInput(rawHeading);

            float scale = 1.0f;
            if(ssom.gamepad1.left_bumper)
                scale = 0.5f;
            else if(ssom.gamepad1.left_trigger > DEAD_ZONE_SIZE)
                scale = 1.0f - ssom.gamepad1.left_trigger * 0.75f;

            f = forward * scale;
            s = lateral * scale;
            t = heading * scale;
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

            if(pose != null) {
                telemetry.addData("f", f);
                telemetry.addData("s", s);
                telemetry.addData("t", t);
                telemetry.addData("X", pose.x());
                telemetry.addData("Y", pose.y());
                telemetry.addData("Heading",Math.toDegrees(pose.heading()));
            }
        }
    }
}
