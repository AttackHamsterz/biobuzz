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

@Autonomous(name = "Auto: All", group = "A")
@Disabled
public class BioBuzzAutoOpMode extends AutoOpMode {

    public static final long OFF_CENTER_START = 1L;
    public static final long CENTER_START = 0L;
    public static final long DISABLE_NON_FLOWER_SHOOT = (1L << 1);
    public static final long NON_FLOWER_SHOOT = 0L;
    public static final long DISABLE_BLOCK = (1L << 2);
    public static final long BLOCK = 0L;
    public static final long VISIT_GARDEN = (1L << 3);
    public static final long VISIT_CLOSE_FLOWER = (1L << 4);
    public static final long VISIT_FAR_FLOWER = (1L << 5);
    public static final long DISABLE_FLOWER_SHOOT = (1L << 6);
    public static final long FLOWER_SHOOT = 0L;
    public static final long PARK_FAR = (1L << 7);
    public static final long PARK_NEAR = 0;
    public static final long VIA_OUTSIDE = (1L << 8);
    public static final long VIA_TUNNEL = 0;
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
    protected long BLOCK_WAIT_MS = 3000;  // wait at the block pose for the hive to tip

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
    protected double flowerBackoffInches = 5.0;

    private Pose startPose;
    private Pose blockPose;
    private Pose nonFlowerShootPose;
    private Pose flowerShootPose;
    private Pose closeFlowerPose;
    private Pose closeExtraFlowerPose;
    private Pose closeFlowerBackoffPose;
    private Pose farFlowerPose;
    private Pose farExtraFlowerPose;
    private Pose farFlowerBackoffPose;
    private Pose nonFlowerSafeTravelPose;
    private Pose flowerSafeTravelPose;
    private Pose parkPose;

    public BioBuzzAutoOpMode(long config) {
        this.ignoreGamepad = true;
        this.config = config;
        this.color = ((BLUE & config) != 0) ? COLOR.BLUE : COLOR.RED;
    }

    private boolean offCenterStart() {
        return (OFF_CENTER_START & config) != 0;
    }

    private boolean disableNonFlowerShoot() {
        return (DISABLE_NON_FLOWER_SHOOT & config) != 0;
    }

    private boolean disableBlock() {
        return (DISABLE_BLOCK & config) != 0;
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
    private boolean disableFlowerShoot() {
        return (DISABLE_FLOWER_SHOOT & config) != 0;
    }

    private boolean farPark() {
        return (PARK_FAR & config) != 0;
    }

    private boolean viaOutside() {
        return (VIA_OUTSIDE & config) != 0;
    }

    // Red is on the left side when facing from the audience and contains 0,0
    // Blue is on the right side when facing from the audience and contains (144, 144)
    @Override
    public void init() {
        // Starting positions
        final double centerStartX = 62.0;
        final double centerStartY = 9.0;
        final double centerStartAngle = 90.0;

        final double offCenterStartX = 38.0;
        final double offCenterStartY = 9.0;
        final double offCenterStartAngle = 90.0;

        // Blocking positions to collect falls
        final double centerBlockX = 60.0;
        final double centerBlockY = 20.0;
        final double centerBlockAngle = 190.0;

        final double offCenterBlockX = 38.0;
        final double offCenterBlockY = 14.0;
        final double offCenterBlockAngle = 160.0;

        // Close flower positions
        final double closeFlowerX = 15;
        final double closeFlowerY = 46.0;
        final double closeFlowerAngle = 180.0;
        final double closeExtraFlowerX = closeFlowerX-2;

        // Far flower positions
        final double farFlowerX = 49.0;
        final double farFlowerY = 127;
        final double farFlowerAngle = 90.0;
        final double farExtraFlowerY = farFlowerY+2;

        // Positions for safe travels
        final double safeNonFlowerOutsideX = 35.0;
        final double safeNonFlowerOutsideY = 35.0;
        final double safeNonFlowerOutsideAngle = 90.0;

        final double safeNonFlowerTunnelX = 60.0;
        final double safeNonFlowerTunnelY = 35.0;
        final double safeNonFlowerTunnelAngle = 90.0;

        final double safeFlowerOutsideX = 35.0;
        final double safeFlowerOutsideY = 110.0;
        final double safeFlowerOutsideAngle = 90.0;

        final double safeFlowerTunnelX = 60.0;
        final double safeFlowerTunnelY = 110.0;
        final double safeFlowerTunnelAngle = 90.0;

        // Parking positions
        final double farParkX = 15;
        final double farParkY = 122.0;
        final double farParkAngle = 270.0;

        final double closeParkX = 15;
        final double closeParkY = 108.0;
        final double closeParkAngle = 270.0;

        if (offCenterStart()) {
            startPose = poseFactory.of(offCenterStartX, offCenterStartY, offCenterStartAngle);
            blockPose = poseFactory.of(offCenterBlockX, offCenterBlockY, offCenterBlockAngle);
            nonFlowerShootPose = poseFactory.of(offCenterStartX, offCenterStartY+1.0, offCenterStartAngle);
        } else {
            startPose = poseFactory.of(centerStartX, centerStartY, centerStartAngle);
            blockPose = poseFactory.of(centerBlockX, centerBlockY, centerBlockAngle);
            nonFlowerShootPose = poseFactory.of(centerStartX, centerStartY+1.0, centerStartAngle);
        }

        // Travel poses
        if (viaOutside()){
            nonFlowerSafeTravelPose = poseFactory.of(safeNonFlowerOutsideX, safeNonFlowerOutsideY, safeNonFlowerOutsideAngle);
            flowerSafeTravelPose = poseFactory.of(safeFlowerOutsideX, safeFlowerOutsideY, safeFlowerOutsideAngle);
            flowerShootPose = poseFactory.of(offCenterStartX, offCenterStartY+1.0, offCenterStartAngle-15.0);
        }else{
            nonFlowerSafeTravelPose = poseFactory.of(safeNonFlowerTunnelX, safeNonFlowerTunnelY, safeNonFlowerTunnelAngle);
            flowerSafeTravelPose = poseFactory.of(safeFlowerTunnelX, safeFlowerTunnelY, safeFlowerTunnelAngle);
            flowerShootPose = poseFactory.of(centerStartX, 144-(offCenterStartY+1.0), centerStartAngle+180.0);
        }

        // Flower poses
        closeFlowerPose = poseFactory.of(closeFlowerX, closeFlowerY, closeFlowerAngle);
        closeExtraFlowerPose = poseFactory.of(closeExtraFlowerX, closeFlowerY, closeFlowerAngle);
        closeFlowerBackoffPose = poseFactory.of(
                closeFlowerX - flowerBackoffInches * Math.cos(Math.toRadians(closeFlowerAngle)),
                closeFlowerY - flowerBackoffInches * Math.sin(Math.toRadians(closeFlowerAngle)),
                closeFlowerAngle);
        farFlowerPose = poseFactory.of(farFlowerX, farFlowerY, farFlowerAngle);
        farExtraFlowerPose = poseFactory.of(farFlowerX, farExtraFlowerY, farFlowerAngle);
        farFlowerBackoffPose = poseFactory.of(
                farFlowerX - flowerBackoffInches * Math.cos(Math.toRadians(farFlowerAngle)),
                farFlowerY - flowerBackoffInches * Math.sin(Math.toRadians(farFlowerAngle)),
                farFlowerAngle);

        // Parking
        if (farPark()) {
            parkPose = poseFactory.of(farParkX, farParkY, farParkAngle);
        } else {
            parkPose = poseFactory.of(closeParkX, closeParkY, closeParkAngle);
        }

        // Initialize
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
    //   10 = backing off the far flower, 11 = safe tunnel
    @Override
    public void buildSchedule() {
        Command driveToNonFlowerShoot = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = nonFlowerShootPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToFlowerShoot = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = flowerShootPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToBlock = lazy(() -> {
            step = 3;
            Pose currentPose = motion.follower.pose();
            Pose nextPose = blockPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToCloseFlower = lazy(() -> {
            step = 5;
            Pose currentPose = motion.follower.pose();
            Pose nextPose = closeFlowerPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToCloseExtraFlower = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = closeExtraFlowerPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command backOffCloseFlower = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = closeFlowerBackoffPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToFarFlower = lazy(() -> {
            step = 6;
            Pose currentPose = motion.follower.pose();
            Pose nextPose = farFlowerPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToFarExtraFlower = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = farExtraFlowerPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command backOffFarFlower = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = farFlowerBackoffPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToPark = lazy(() -> {
            step = 7;
            Pose currentPose = motion.follower.pose();
            Pose nextPose = parkPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToNonFlowerTravel = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = nonFlowerSafeTravelPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToFlowerTravel = lazy(() -> {
            Pose currentPose = motion.follower.pose();
            Pose nextPose = flowerSafeTravelPose;
            return follow(motion.follower, line(currentPose, nextPose).linear(currentPose, nextPose, turnEndFraction));
        });
        Command driveToGarden = lazy(() -> {
            step = 4;
            Pose currentPose = motion.follower.pose();
            Path dynamicPath = line(currentPose, closeFlowerPose).linear(currentPose, closeFlowerPose, turnEndFraction);
            return follow(motion.follower, dynamicPath);
        });
        
        Command nothing = instant(() -> {});
        Command startIntake = instant(() -> intake.setPower(1.0));
        Command stopIntake = instant(() -> intake.setPower(0.0));
        Command startFlywheel = instant(() -> launcher.setRPMFromDistance(1.0,0,0));
        Command standbyFlywheel = instant(() -> launcher.setRPMFromDistance(0.5,0,0));
        Command allowPollen = instant(() -> ballStopper.allowPollen());
        Command allowNectar = instant(() -> ballStopper.allowNectar());
        Command allowAll = instant(() -> ballStopper.allowAll());
        Command stopAll = instant(()-> ballStopper.stopAll());
        Command runLauncher = startFlywheel.until(() -> launcher.launchReady());
        Command launchAll = sequential(runLauncher, allowAll, waitMs(3000), stopAll, standbyFlywheel);

        Command zeroSwerve = race(repeat(instant(() -> ((HamFollower) motion.follower).zero()), 1000),
                waitMs(ZERO_DURATION_MS));
        Command zeroAuto = sequential(
                waitMs(ZERO_START_MS),
                instant(() -> step = 9),
                zeroSwerve);
        Command startDelay = sequential(
                instant(() -> step = 1),
                waitMs(startDelayMs));

        Command maybeNonFlowerShoot = conditional(this::disableNonFlowerShoot,
                nothing,
                sequential(driveToNonFlowerShoot, launchAll));
        Command maybeFlowerShoot = conditional(this::disableFlowerShoot,
                nothing,
                sequential(driveToFlowerShoot, launchAll));

        Command closeFlowerIntake = sequential(startIntake, race(waitMs(3000), driveToCloseExtraFlower), stopIntake);
        Command farFlowerIntake = sequential(startIntake, race(waitMs(3000), driveToFarExtraFlower), stopIntake);
        Command maybeBlock = conditional(this::disableBlock,
                nothing,
                sequential(
                        parallel(driveToBlock, startIntake),
                        // Wait for hive to tip, either timer or fiducial reading, and wait BLOCK_WAIT_MS
                        waitMs(BLOCK_WAIT_MS),
                        stopIntake));
        Command travelNonFlowerToFlower = sequential(driveToNonFlowerTravel, driveToFlowerTravel);
        Command travelFlowerToNonFlower = sequential(driveToFlowerTravel, driveToNonFlowerTravel);

        Command maybeGarden = conditional(this::visitGarden,
                driveToGarden,
                nothing);
        Command maybeCloseFlower = conditional(this::visitCloseFlower,
                sequential(driveToCloseFlower, settleAt(closeFlowerPose), closeFlowerIntake),
                nothing);
        Command maybeFarFlower = conditional(this::visitFarFlower,
                sequential(driveToFarFlower, settleAt(farFlowerPose), farFlowerIntake,
                        backOffFarFlower),
                nothing);

        // The overall schedule
        schedule(
                race(
                        // No matter what we zero pod wheels before auto ends
                        zeroAuto,
                        // Normal schedule
                        sequential(
                                standbyFlywheel,
                                startDelay,
                                maybeNonFlowerShoot,
                                maybeBlock,
                                //check hive drop
                                maybeGarden,
                                maybeCloseFlower,
                                travelNonFlowerToFlower,
                                maybeFlowerShoot,
                                maybeFarFlower,
                                maybeFlowerShoot,
                                driveToPark,
                                // Settle before zeroing: zeroing stops the hold, so any error left here stays
                                settleAt(parkPose),
                                waitMs(pointPauseMs),
                                instant(() -> step = 99),
                                zeroSwerve)));
    }
}
