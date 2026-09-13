package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Auto: Close", group = "Robot")
@Disabled
public class CloseAutoOpMode extends AutoOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose firstScorePose;
    private Pose secondScorePose;
    private Pose thirdScorePose;
    private Pose fourthScorePose;
    private Pose parkPose;

    @Override public void init() {
        final double centerLineX = 72.0;
        final double startPoseX = (color == COLOR.BLUE) ? centerLineX-10 : centerLineX+10;
        final double startPoseY = 72.0;
        final double startPoseAngle = (color == COLOR.BLUE) ? 0 : 180;

        final double firstScorePoseX = (color == COLOR.BLUE) ? centerLineX-30.0 : centerLineX+30;
        final double firstScorePoseY = 80.0;
        final double firstScorePoseAngle = (color == COLOR.BLUE) ? 45 : 135;

        final double secondScorePoseX = (color == COLOR.BLUE) ? centerLineX-27.0 : centerLineX+27;
        final double secondScorePoseY = 40.0;
        final double secondScorePoseAngle = -90.0;

        final double thirdScorePoseX = (color == COLOR.BLUE) ? centerLineX-20.0 : centerLineX+20;
        final double thirdScorePoseY = 60.0;
        final double thirdScorePoseAngle = (color == COLOR.BLUE) ? 180: 0;

        final double fourthScorePoseX = (color == COLOR.BLUE) ? centerLineX-90.0 : centerLineX+90;
        final double fourthScorePoseY = 30.0;
        final double fourthScorePoseAngle = (color == COLOR.BLUE) ? 130: 50;

        final double parkX = (color == COLOR.BLUE) ? centerLineX-15.0 : centerLineX+15;
        final double parkY = 30.0;
        final double parkAngle = 90.0;

        firstScorePose = poseFactory.of(firstScorePoseX, firstScorePoseY, firstScorePoseAngle);
        secondScorePose = poseFactory.of(secondScorePoseX, secondScorePoseY, secondScorePoseAngle);
        thirdScorePose = poseFactory.of(thirdScorePoseX, thirdScorePoseY, thirdScorePoseAngle);
        fourthScorePose = poseFactory.of(fourthScorePoseX, fourthScorePoseY, fourthScorePoseAngle);
        parkPose = poseFactory.of(parkX, parkY, parkAngle);

        // setup and build the paths
        super.init();
        motion.follower.setPose(poseFactory.of(startPoseX, startPoseY, startPoseAngle));
    }

    private Path firstScore() {
        Pose currentPose = motion.follower.pose();
        return line(currentPose, firstScorePose).linear(currentPose, firstScorePose);
    }

    private Path secondScore() {
        Pose currentPose = motion.follower.pose();
        return line(currentPose, secondScorePose).linear(currentPose, secondScorePose);
    }

    private Path thirdScore() {
        Pose currentPose = motion.follower.pose();
        return line(currentPose, thirdScorePose).linear(currentPose, thirdScorePose);
    }

    private Path fourthScore() {
        Pose currentPose = motion.follower.pose();
        return line(currentPose, fourthScorePose).linear(currentPose, fourthScorePose);
    }

    private Path park() {
        Pose currentPose = motion.follower.pose();
        return line(currentPose, parkPose).linear(currentPose, parkPose);
    }

    Command startIntake = instant(() -> intake.setPower(1.0));
    Command stopIntake = instant(() -> intake.setPower(0.0));

    @Override
    public void buildSchedule() {
        schedule(follow(motion.follower, firstScore()));
        schedule(startIntake);
        schedule(follow(motion.follower, secondScore()));
        schedule(stopIntake);
        schedule(follow(motion.follower, thirdScore()));
        schedule(follow(motion.follower, fourthScore()));
        schedule(follow(motion.follower, park()));
    }
}
