package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Close", group = "Robot")
@Disabled
public class CloseAutoOpMode extends AutoOpMode {

    private Pose startPose;
    private Pose parkPose;
    private Pose initialScorePose;

    @Override public void init() {
        final double centerLineX = 72.0;
        final double startPoseX = 0;
        final double startPoseY = 0;
        final double initialScorePoseX = 0;
        final double initialScorePoseY = 0;
        final double parkX = 0;
        final double parkY = 0;


        startPose = new Pose((color == COLOR.BLUE) ? centerLineX-startPoseX :centerLineX+startPoseX, startPoseY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        initialScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-initialScorePoseX :centerLineX+initialScorePoseX, initialScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        parkPose = new Pose((color == COLOR.BLUE) ? centerLineX-parkX :centerLineX+parkX, parkY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));


        // setup and build the paths
        super.init();
        //motion.follower.setStartingPose(startPose);
    }

    @Override
    public void buildPaths() {
        //put the pathing here :)
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //something happens
                incrementPathState();
                break;

            case 1:
                //something else happens
                incrementPathState();
                break;
        }

    }

}
