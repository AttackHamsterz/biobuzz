package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Close", group = "Robot")
@Disabled
public class CloseAutoOpMode extends AutoOpMode {

    private Pose startPose;
    private Pose firstScorePose;
    private Pose secondScorePose;
    private Pose thirdScorePose;
    private Pose fourthScorePose;
    private Pose parkPose;
    private PathChain startToFirstPath;
    private PathChain firstToSecondPath;
    private PathChain secondToThirdPath;
    private PathChain thirdToFourthPath;
    private PathChain fourthToParkPath;

    @Override public void init() {
        final double centerLineX = 72.0;
        final double startPoseX = 72.0-9.0;
        final double startPoseY = 72.0;
        final double firstScorePoseX = 72.0-30.0;
        final double firstScorePoseY = 80.0;
        final double secondScorePoseX = 72.0-27.0;
        final double secondScorePoseY = 40.0;
        final double thirdScorePoseX = 72.0-20.0;
        final double thirdScorePoseY = 60.0;
        final double fourthScorePoseX = 72.0-90.0;
        final double fourthScorePoseY = 30.0;
        final double parkX = 72.0-15.0;
        final double parkY = 30.0;


        startPose = new Pose((color == COLOR.BLUE) ? centerLineX-startPoseX :centerLineX+startPoseX, startPoseY, Math.toRadians((color == COLOR.BLUE) ? 0 : 180));
        firstScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-firstScorePoseX :centerLineX+firstScorePoseX, firstScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 45 : 135));
        secondScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-secondScorePoseX :centerLineX+secondScorePoseX, secondScorePoseY, Math.toRadians((color == COLOR.BLUE) ? -90: -90));
        thirdScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-thirdScorePoseX :centerLineX+thirdScorePoseX, thirdScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 180: 0));
        fourthScorePose = new Pose((color == COLOR.BLUE) ? centerLineX-fourthScorePoseX :centerLineX+fourthScorePoseX, fourthScorePoseY, Math.toRadians((color == COLOR.BLUE) ? 130: 50));
        parkPose = new Pose((color == COLOR.BLUE) ? centerLineX-parkX :centerLineX+parkX, parkY, Math.toRadians((color == COLOR.BLUE) ? 90 : 90));


        // setup and build the paths
        super.init();
        motion.follower.setStartingPose(startPose);
    }

    @Override
    public void buildPaths() {
        startToFirstPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(startPose,firstScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstScorePose.getHeading())
                .build();
        firstToSecondPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, secondScorePose))
                .setLinearHeadingInterpolation(firstScorePose.getHeading(), secondScorePose.getHeading())
                .build();
        secondToThirdPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, thirdScorePose))
                .setLinearHeadingInterpolation(secondScorePose.getHeading(), thirdScorePose.getHeading())
                .build();
        thirdToFourthPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, fourthScorePose))
                .setLinearHeadingInterpolation(thirdScorePose.getHeading(), fourthScorePose.getHeading())
                .build();
        fourthToParkPath = motion.follower.pathBuilder()
                .addPath(new BezierLine(motion.follower::getPose, parkPose))
                .setLinearHeadingInterpolation(fourthScorePose.getHeading(), parkPose.getHeading())
                .build();
        setPathState(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                motion.follower.followPath(startToFirstPath, true);
                incrementPathState();
                break;
            case 1:
                if(!motion.follower.isBusy()) {
                    motion.follower.followPath(firstToSecondPath, true);
                    incrementPathState();
                }
                break;
            case 2:
                if(!motion.follower.isBusy()) {
                    motion.follower.followPath(secondToThirdPath, true);
                    incrementPathState();
                }
                break;
            case 3:
                if(!motion.follower.isBusy()) {
                    motion.follower.followPath(thirdToFourthPath, true);
                    incrementPathState();
                }
                break;
            case 4:
                if(!motion.follower.isBusy()) {
                    motion.follower.followPath(fourthToParkPath, true);
                    incrementPathState();
                }
                break;
        }

    }

}
