package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Swerve;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Motion extends RobotPart {
    public Follower follower;
    private double externalTurn;
    private double f, s, t;

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

    @Override
    public void loop() {
        if(!ssom.ignoreGamepad) {
            if (ssom.gamepad1.start)
            {
                follower.manual(0,0,0);
                return;
            }
            float scale = 1.0f;
            if(ssom.gamepad1.left_bumper)
                scale = 0.5f;
            else if(ssom.gamepad1.left_trigger > 0.05)
                scale = 1.0f - ssom.gamepad1.left_trigger * 0.75f;

            f = -ssom.gamepad1.left_stick_y*scale;
            s = ssom.gamepad1.left_stick_x*scale;
            t = ssom.gamepad1.right_stick_x*scale;

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
