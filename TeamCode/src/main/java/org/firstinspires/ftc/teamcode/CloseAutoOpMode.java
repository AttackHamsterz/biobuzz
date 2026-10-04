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
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.HamFollower;

@Autonomous(name = "Auto: Close", group = "Robot")
@Disabled
public class CloseAutoOpMode extends AutoOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose startPose;
    private Pose firstScorePose;
    private Pose secondScorePose;
    private Pose thirdScorePose;
    private Pose fourthScorePose;
    private Pose parkPose;

    // Red is on the left side when facing from the audience and contains 0,0
    // Blue is on the right side when facing from the audience and contains (144, 144)
    @Override public void init() {
        final double centerLineX = 72.0;
        final double startPoseX = (color == COLOR.RED) ? centerLineX-10 : centerLineX+10;
        final double startPoseY = 9.0;
        final double startPoseAngle = 90.0;

        final double firstScorePoseX = (color == COLOR.RED) ? centerLineX-10.0 : centerLineX+10;
        final double firstScorePoseY = 20.0;
        final double firstScorePoseAngle = 90.0;

        final double secondScorePoseX = (color == COLOR.RED) ? centerLineX-10.0 : centerLineX+10;
        final double secondScorePoseY = 40.0;
        final double secondScorePoseAngle = 90.0;

        final double thirdScorePoseX = (color == COLOR.RED) ? centerLineX-30.0 : centerLineX+30;
        final double thirdScorePoseY = 40.0;
        final double thirdScorePoseAngle = 90.0;

        final double fourthScorePoseX = (color == COLOR.RED) ? centerLineX-10.0 : centerLineX+10;
        final double fourthScorePoseY = 40.0;
        final double fourthScorePoseAngle = 180.0;

        final double parkX = (color == COLOR.RED) ? centerLineX-10.0 : centerLineX+10;
        final double parkY = 20.0;
        final double parkAngle =  180.0;

        startPose = poseFactory.of(startPoseX, startPoseY, startPoseAngle);
        firstScorePose = poseFactory.of(firstScorePoseX, firstScorePoseY, firstScorePoseAngle);
        secondScorePose = poseFactory.of(secondScorePoseX, secondScorePoseY, secondScorePoseAngle);
        thirdScorePose = poseFactory.of(thirdScorePoseX, thirdScorePoseY, thirdScorePoseAngle);
        fourthScorePose = poseFactory.of(fourthScorePoseX, fourthScorePoseY, fourthScorePoseAngle);
        parkPose = poseFactory.of(parkX, parkY, parkAngle);

        super.init();
        motion.follower.setPose(startPose);
    }

    // Each path starts at the previous fixed pose (not the live pose),
    // so every run drives the same route

    private Path firstScore() {
        return line(startPose, firstScorePose).linear(startPose, firstScorePose);
    }

    private Path secondScore() {
        return line(firstScorePose, secondScorePose).linear(firstScorePose, secondScorePose);
    }

    private Path thirdScore() {
        return line(secondScorePose, thirdScorePose).linear(secondScorePose, thirdScorePose);
    }

    private Path fourthScore() {
        return line(thirdScorePose, fourthScorePose).linear(thirdScorePose, fourthScorePose);
    }

    private Path park() {
        return line(fourthScorePose, parkPose).linear(fourthScorePose, parkPose);
    }

    Command startIntake = instant(() -> intake.setPower(1.0));
    Command stopIntake = instant(() -> intake.setPower(0.0));
    Command zeroSwerve = instant(() -> ((HamFollower)motion.follower).zero());

    @Override
    public void buildSchedule() {
        schedule(sequential(
                parallel(follow(motion.follower, firstScore()),
                    startIntake),
                parallel(follow(motion.follower, secondScore()),
                    stopIntake),
                follow(motion.follower, thirdScore()),
                follow(motion.follower, fourthScore()),
                follow(motion.follower, park()),
                race(repeat(zeroSwerve, 1000),
                    waitMs(2000))
        ));
    }
}
