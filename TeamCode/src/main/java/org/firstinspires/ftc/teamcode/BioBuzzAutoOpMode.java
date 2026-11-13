package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.pedro.HamFollower;

@Autonomous(name = "Auto: Close", group = "Robot")
@Disabled
public class BioBuzzAutoOpMode extends AutoOpMode {

    public static final long CENTER_START = 0L;
    public static final long OFF_CENTER_START = 1L;
    public static final long SHOOT_FIRST = (1L << 1);
    public static final long BLOCK_FIRST_DROP = (1L << 2);
    public static final long VISIT_GARDEN = (1L << 3);
    public static final long VISIT_CLOSE_FLOWER = (1L << 4);
    public static final long VISIT_FAR_FLOWER = (1L << 5);
    public static final long SHOOT_SECOND = (1L << 6);
    public static final long FAR_PARK = (1L << 7);
    public static final long BLUE = (1L << 62);

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final long config;

    private Pose startPose;
    private Pose blockPose;
    private Pose closeFlowerPose;
    private Pose farFlowerPose;
    private Pose parkPose;

    public BioBuzzAutoOpMode(long config) {
        this.ignoreGamepad = true;
        this.config = config;
        this.color = ((BLUE & config) != 0) ? COLOR.BLUE : COLOR.RED;
    }

    private boolean offCenterStart() {
        return (OFF_CENTER_START & config) != 0;
    }

    private boolean shootFirst() {
        return (SHOOT_FIRST & config) != 0;
    }

    private boolean blockFirstDrop() {
        return (BLOCK_FIRST_DROP & config) != 0;
    }

    private boolean visitGarden() {
        return (VISIT_GARDEN & config) != 0;
    }

    private boolean visitCloseFlower() {
        return (VISIT_CLOSE_FLOWER & config) != 0;
    }

    private boolean visitFarFlower() {
        return (VISIT_FAR_FLOWER & config) != 0;
    }

    private boolean farPark() {
        return (FAR_PARK & config) != 0;
    }

    // Red is on the left side when facing from the audience and contains 0,0
    // Blue is on the right side when facing from the audience and contains (144, 144)
    @Override
    public void init() {
        final double centerStartPoseX = 62.0;
        final double centerStartPoseY = 9.0;
        final double centerStartPoseAngle = 90.0;

        final double offCenterStartPoseX = 38.0;
        final double offCenterStartPoseY = 9.0;
        final double offCenterStartPoseAngle = 90.0;

        final double centerBlockPoseX = 62.0;
        final double centerBlockPoseY = 27.0;
        final double centerBlockPoseAngle = 110.0;

        final double offCenterBlockPoseX = 38.0;
        final double offCenterBlockPoseY = 27.0;
        final double offCenterBlockPoseAngle = 70.0;

        final double closeFlowerPoseX = 18.0;
        final double closeFlowerPoseY = 48.0;
        final double closeFlowerPoseAngle = 270.0;

        final double farFlowerPoseX = 49.5;
        final double farFlowerPoseY = 126.0;
        final double farFlowerPoseAngle = 180.0;

        final double farParkX = 15;
        final double farParkY = 94.0;
        final double farParkAngle = 270.0;

        final double closeParkX = 15;
        final double closeParkY = 94.0;
        final double closeParkAngle = 270.0;

        if (offCenterStart()) {
            startPose = poseFactory.of(offCenterStartPoseX, offCenterStartPoseY, offCenterStartPoseAngle);
            blockPose = poseFactory.of(offCenterBlockPoseX, offCenterBlockPoseY, offCenterBlockPoseAngle);
        } else {
            startPose = poseFactory.of(centerStartPoseX, centerStartPoseY, centerStartPoseAngle);
            blockPose = poseFactory.of(centerBlockPoseX, centerBlockPoseY, centerBlockPoseAngle);
        }

        closeFlowerPose = poseFactory.of(closeFlowerPoseX, closeFlowerPoseY, closeFlowerPoseAngle);
        farFlowerPose = poseFactory.of(farFlowerPoseX, farFlowerPoseY, farFlowerPoseAngle);

        if (farPark()) {
            parkPose = poseFactory.of(farParkX, farParkY, farParkAngle);
        } else {
            parkPose = poseFactory.of(closeParkX, closeParkY, closeParkAngle);
        }

        super.init();
        motion.follower.setPose(startPose);
    }

    @Override
    public void buildSchedule() {
        Command driveToBlock = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, blockPose).linear(currentPose, blockPose);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToCloseFlower = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, closeFlowerPose).linear(currentPose, closeFlowerPose);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToFarFlower = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, farFlowerPose).linear(currentPose, farFlowerPose);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToPark = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, parkPose).linear(currentPose, parkPose);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToGarden = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, closeFlowerPose).linear(currentPose, closeFlowerPose);
            return follow(motion.follower, dynamicPath);
        });
        Command nothing = instant(() -> {
        });
        Command startIntake = instant(() -> intake.setPower(1.0));
        Command stopIntake = instant(() -> intake.setPower(0.0));
        Command zeroSwerve = race(repeat(instant(() -> ((HamFollower) motion.follower).zero()), 1000),
                waitMs(2000));
        Command zeroAuto = sequential(
                waitMs(28000),
                zeroSwerve);
        Command maybeShoot = conditional(this::shootFirst,
                startIntake,
                nothing);
        Command maybeBlock = conditional(this::blockFirstDrop,
                sequential(
                        parallel(driveToBlock, startIntake),
                        // Wait for hive to tip, either timer or fiducial reading, and wait 2 secs to block
                        waitMs(3000)),
                nothing);
        Command maybeGarden = conditional(this::visitGarden,
                driveToGarden,
                nothing);
        Command maybeCloseFlower = conditional(this::visitCloseFlower,
                driveToCloseFlower,
                nothing);
        Command maybeFarFlower = conditional(this::visitFarFlower,
                driveToFarFlower,
                nothing);

        // The overall schedule
        schedule(
                race(
                        // No matter what we zero pod wheels before auto ends
                        zeroAuto,
                        // Normal schedule
                        sequential(
                                maybeShoot,
                                maybeBlock,
                                maybeGarden,
                                maybeCloseFlower,
                                maybeFarFlower,
                                driveToPark,
                                zeroSwerve)));
    }
}
