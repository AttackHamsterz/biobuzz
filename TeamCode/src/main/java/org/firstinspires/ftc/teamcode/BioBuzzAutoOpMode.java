package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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
    public static final long RED = 0;

    // Pods must be zeroed before auto ends (30 s)
    private static final long ZERO_START_MS = 28000;
    private static final long ZERO_DURATION_MS = 2000;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final long config;

    // Timings in ms. Read when start() builds the schedule, so anything that
    // sets them during INIT (a subclass or a Driver Station selector) takes effect.
    protected long startDelayMs = 0;    // wait before moving, to stay clear of a partner
    protected long blockWaitMs = 3000;  // wait at the block pose for the hive to tip

    // Settle check: how close counts as "arrived" before a mechanism acts. Pedro's follow()
    // finishes at the end of the path, while the robot is still moving, so steps that need the
    // robot stopped and aimed wait on this. Must stay above the hold deadbands (about 2.8 deg
    // heading, 0.73 in strafe) or it will always run to the timeout.
    protected double settleInches = 1.0;
    protected double settleDegrees = 4.0;
    protected long settleTimeoutMs = 1000;

    // Diagnostic pause at each point (close flower, far flower, park), so positions can be
    // checked on the field and the log has a still pose at each one. The robot holds the pose
    // while it waits. Set to 0 for match runs.
    protected long pointPauseMs = 2000;

    // How far along each drive the turn finishes (0-1). Pedro turns over this fraction of the
    // path, then holds the final heading, so the robot arrives already aimed instead of still
    // turning. 1.0 spreads the turn over the whole path (the old behavior); lower turns faster.
    protected double turnEndFraction = 0.8;

    // How far to back straight off the far flower before turning toward park. The robot's front
    // sits about 2 in from the flower there, and a rotated 18 in robot's corners reach 12.7 in
    // from its center instead of 9, so turning in place swings them into the flower. Read in
    // init(), where the back-off pose is built.
    protected double farFlowerBackoffInches = 5.0;

    private Pose startPose;
    private Pose blockPose;
    private Pose closeFlowerPose;
    private Pose farFlowerPose;
    private Pose farFlowerBackoffPose;
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
        final double centerBlockPoseY = 21.0;
        final double centerBlockPoseAngle = 190.0;

        final double offCenterBlockPoseX = 38.0;
        final double offCenterBlockPoseY = 21.0;
        final double offCenterBlockPoseAngle = 170.0;

        final double closeFlowerPoseX = 14.5;
        final double closeFlowerPoseY = 46.0;
        final double closeFlowerPoseAngle = 180.0;

        final double farFlowerPoseX = 49.0;
        final double farFlowerPoseY = 125.5;
        final double farFlowerPoseAngle = 90.0;

        final double farParkX = 15;
        final double farParkY = 122.0;
        final double farParkAngle = 270.0;

        final double closeParkX = 15;
        final double closeParkY = 108.0;
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
        // Straight back from the far flower, opposite the way the robot faces there
        farFlowerBackoffPose = poseFactory.of(
                farFlowerPoseX - farFlowerBackoffInches * Math.cos(Math.toRadians(farFlowerPoseAngle)),
                farFlowerPoseY - farFlowerBackoffInches * Math.sin(Math.toRadians(farFlowerPoseAngle)),
                farFlowerPoseAngle);

        if (farPark()) {
            parkPose = poseFactory.of(farParkX, farParkY, farParkAngle);
        } else {
            parkPose = poseFactory.of(closeParkX, closeParkY, closeParkAngle);
        }

        super.init();
        motion.follower.setPose(startPose);
    }

    private boolean atPose(Pose target) {
        Pose p = motion.follower.pose();
        double dist = Math.hypot(p.x() - target.x(), p.y() - target.y());
        double dh = Math.toDegrees(p.heading() - target.heading());
        dh = ((dh + 180) % 360 + 360) % 360 - 180;   // wrap to -180..180
        return dist <= settleInches && Math.abs(dh) <= settleDegrees;
    }

    // Wait until arrived, or the timeout, whichever comes first
    private Command settleAt(Pose target) {
        return race(waitUntil(() -> atPose(target)), waitMs(settleTimeoutMs));
    }

    // Step numbers, written to the run log's leg column (see AutoOpMode). Fixed per step,
    // not counted, so the same step has the same number in every config:
    //   1 = start delay, 3 = block (drive + wait), 4 = garden, 5 = close flower,
    //   6 = far flower, 7 = park, 8 = zeroing after park, 9 = zeroing forced by the timer,
    //   10 = backing off the far flower
    @Override
    public void buildSchedule() {
        Command driveToBlock = lazy(() -> {
            step = 3;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, blockPose).linear(currentPose, blockPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToCloseFlower = lazy(() -> {
            step = 5;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, closeFlowerPose).linear(currentPose, closeFlowerPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToFarFlower = lazy(() -> {
            step = 6;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, farFlowerPose).linear(currentPose, farFlowerPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        Command backOffFarFlower = lazy(() -> {
            step = 10;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, farFlowerBackoffPose).linear(currentPose, farFlowerBackoffPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToPark = lazy(() -> {
            step = 7;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, parkPose).linear(currentPose, parkPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        Command driveToGarden = lazy(() -> {
            step = 4;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, closeFlowerPose).linear(currentPose, closeFlowerPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        Command nothing = instant(() -> {
        });
        Command startIntake = instant(() -> intake.setPower(1.0));
        Command stopIntake = instant(() -> intake.setPower(0.0));
        Command zeroSwerve = race(repeat(instant(() -> ((HamFollower) motion.follower).zero()), 1000),
                waitMs(ZERO_DURATION_MS));
        Command zeroAuto = sequential(
                waitMs(ZERO_START_MS),
                instant(() -> step = 9),
                zeroSwerve);
        Command startDelay = sequential(
                instant(() -> step = 1),
                waitMs(startDelayMs));
        Command maybeShoot = conditional(this::shootFirst,
                startIntake,
                nothing);
        Command maybeBlock = conditional(this::blockFirstDrop,
                sequential(
                        parallel(driveToBlock, startIntake),
                        // Wait for hive to tip, either timer or fiducial reading, and wait 2 secs to block
                        waitMs(blockWaitMs)),
                nothing);
        Command maybeGarden = conditional(this::visitGarden,
                driveToGarden,
                nothing);
        Command maybeCloseFlower = conditional(this::visitCloseFlower,
                sequential(driveToCloseFlower, settleAt(closeFlowerPose), waitMs(pointPauseMs)),
                nothing);
        Command maybeFarFlower = conditional(this::visitFarFlower,
                sequential(driveToFarFlower, settleAt(farFlowerPose), waitMs(pointPauseMs),
                        backOffFarFlower),
                nothing);

        // The overall schedule
        schedule(
                race(
                        // No matter what we zero pod wheels before auto ends
                        zeroAuto,
                        // Normal schedule
                        sequential(
                                startDelay,
                                maybeShoot,
                                maybeBlock,
                                maybeGarden,
                                maybeCloseFlower,
                                maybeFarFlower,
                                driveToPark,
                                // Settle before zeroing: zeroing stops the hold, so any error left here stays
                                settleAt(parkPose),
                                waitMs(pointPauseMs),
                                instant(() -> step = 8),
                                zeroSwerve)));
    }
}
