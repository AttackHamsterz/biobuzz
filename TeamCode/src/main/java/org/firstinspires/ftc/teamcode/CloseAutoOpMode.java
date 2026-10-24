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

    private Pose centerStartPose;
    private Pose blockPose;
    private Pose closeFlowerPose;
    private Pose farFlowerPose;
    private Pose fourthScorePose;
    //fourth score pose doesnt have a use yet btw
    private Pose parkPose;
    private double flowerOffset = 1;
    private long stepFilter = 0;
    //1 = block pose



    // Red is on the left side when facing from the audience and contains 0,0
    // Blue is on the right side when facing from the audience and contains (144, 144)
    @Override public void init() {
        final double centerLineX = 72.0;
        final double centerStartPoseX = 62.0;
        final double centerStartPoseY = 9.0;
        final double centerStartPoseAngle = 90.0;

        final double blockPoseX = 62.0;
        final double blockPoseY = 27.0;
        final double blockPoseAngle = 110.0;

        final double closeFlowerPoseX = 18.0;
        final double closeFlowerPoseY = 47.0+flowerOffset;
        final double closeFlowerPoseAngle = 270.0;

        final double farFlowerPoseX = 48.5+flowerOffset;
        final double farFlowerPoseY = 126.0;
        final double farFlowerPoseAngle = 180.0;

        final double fourthScorePoseX = (color == COLOR.RED) ? centerLineX-10.0 : centerLineX+10;
        final double fourthScorePoseY = 50.0;
        final double fourthScorePoseAngle = 180.0;

        final double parkX = 15;
        final double parkY = 94.0;
        final double parkAngle =  270.0;

        centerStartPose = poseFactory.of(centerStartPoseX, centerStartPoseY, centerStartPoseAngle);
        blockPose = poseFactory.of(blockPoseX, blockPoseY, blockPoseAngle);
        closeFlowerPose = poseFactory.of(closeFlowerPoseX, closeFlowerPoseY, closeFlowerPoseAngle);
        farFlowerPose = poseFactory.of(farFlowerPoseX, farFlowerPoseY, farFlowerPoseAngle);
        fourthScorePose = poseFactory.of(fourthScorePoseX, fourthScorePoseY, fourthScorePoseAngle);
        parkPose = poseFactory.of(parkX, parkY, parkAngle);

        super.init();
        motion.follower.setPose(centerStartPose);
    }

    // Each path starts at the previous fixed pose (not the live pose),
    // so every run drives the same route

    private Path centerStartToBlock() {
        return line(centerStartPose, blockPose).linear(centerStartPose, blockPose);
    }
    private Path blockToCloseFlower() {
        return line(blockPose, closeFlowerPose).linear(blockPose, closeFlowerPose);
    }
    private Path closeFlowerToBlock() {
        return line(closeFlowerPose, blockPose).linear(closeFlowerPose, blockPose);
    }
    private Path closeFlowerToFarFlower() {
        return line(closeFlowerPose, farFlowerPose).linear(closeFlowerPose, farFlowerPose);
    }
    private Path farFlowerToCloseFlower() {
        return line(farFlowerPose, closeFlowerPose).linear(farFlowerPose, closeFlowerPose);
    }
    private Path closeFlowerToPark() {
        return line(closeFlowerPose, parkPose).linear(closeFlowerPose, parkPose);
    }
    private Path farFlowerToPark() {
        return line(farFlowerPose, parkPose).linear(farFlowerPose, parkPose);
    }

    Command startIntake = instant(() -> intake.setPower(1.0));
    Command stopIntake = instant(() -> intake.setPower(0.0));
    Command zeroSwerve = instant(() -> ((HamFollower)motion.follower).zero());


    @Override
    public void buildSchedule() {
        schedule(sequential(
                //shoot 4 preloaded pollen
                parallel(follow(motion.follower, centerStartToBlock()),
                    startIntake),
                //wait for hive to tip, either timer or fiducial reading, and wait 2 secs to block
                parallel(follow(motion.follower, blockToCloseFlower()),
                    stopIntake),
                //intake from close flower
                follow(motion.follower, closeFlowerToFarFlower()),
                //shoot from the far flower
                //intake from far flower
                //shoot from the far flower
                follow(motion.follower, farFlowerToPark()),
                race(repeat(zeroSwerve, 1000),
                    waitMs(2000))
        ));
    }
}
