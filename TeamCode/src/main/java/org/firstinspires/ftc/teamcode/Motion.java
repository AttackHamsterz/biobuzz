package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Motion extends RobotPart {
    public Follower follower;
    private double externalTurn;
    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        follower = Constants.createFollower(ssom.hardwareMap);
        follower.activateAllPIDFs();
        externalTurn = 0;
    }
    public void setTurn(double turn) { externalTurn = turn; }

    public void init() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        if(!ssom.gamepadBuffer.ignoreGamepad) {
            float scale = 1.0f;
            if(ssom.gamepadBuffer.g1LeftBumper)
                scale = 0.5f;
            else if(ssom.gamepadBuffer.g1LeftTrigger > 0.05)
                scale = 1.0f - ssom.gamepadBuffer.g1LeftTrigger * 0.75f;

            double f = -ssom.gamepadBuffer.g1LeftStickY*scale;
            double s = -ssom.gamepadBuffer.g1LeftStickX*scale;
            double t = -externalTurn-ssom.gamepadBuffer.g1RightStickX*scale;

            follower.setTeleOpDrive(f, s, t, true);
            follower.update();
        }
        //stop all motion->
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
        follower.update();
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 1) !=0) {
            Pose pose = follower.getPose();
            if(pose != null) {
                telemetry.addData("X", pose.getX());
                telemetry.addData("Y", pose.getY());
                telemetry.addData("Heading",Math.toDegrees(pose.getHeading()));
            }
        }

    }
}
