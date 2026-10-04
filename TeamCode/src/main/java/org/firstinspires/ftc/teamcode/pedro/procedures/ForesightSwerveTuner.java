package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.*;
import java.util.function.Function;

import static com.pedropathing.utils.Utils.linearFit;
import static com.pedropathing.utils.Utils.quadraticFit;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class ForesightSwerveTuner extends Procedure {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public ForesightSwerveTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Foresight Swerve Tuner", "A procedure for tuning the Foresight Algorithm for swerve.");
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    public void run() throws InterruptedException {
        int tunerID = 0;

        do {
            Inputs tunerSelection = inputs("Tuner", "0 (max v), 1 (deccel), 2 (forward kp), 3 (heading kp), 4 (heading braking), 5 (forward braking), 6 (quit)");
            Inputs.Field<Double> tuner = tunerSelection.d("Tuner").withDefault(0.0);
            awaitInputs(tunerSelection);
            tunerID = tuner.get().intValue();
            switch (tunerID) {
                case 0:
                    Inputs distanceInput = inputs("Distance", "The distance to drive in inches for the Max Achievable Forward and Strafe Identifiers");
                    Inputs.Field<Double> distance = distanceInput.d("Distance").withDefault(48.0);
                    awaitInputs(distanceInput);

                    double forwardVelocity = runOpMode(new ForwardVelocity(localizerFunction, drivetrainFunction, distance.get()));
                    result("maxAchievableForwardVelocity", forwardVelocity);
                    result("maxAchievableStrafeVelocity", forwardVelocity);

                    code(Language.JAVA,
                                    "                c.maxAchievableForwardVelocity.set(" + forwardVelocity + ");\n" +
                                    "                c.maxAchievableStrafeVelocity.set(" + forwardVelocity + ");\n" );

                    break;
                case 1:
                    Inputs velocityInput = inputs("Velocity", "The velocity to drive to in inches for the Max Achievable Forward and Strafe Deceleration Identifiers");
                    Inputs.Field<Double> velocity = velocityInput.d("Velocity").withDefault(30.0);
                    awaitInputs(velocityInput);

                    double forwardDeceleration = runOpMode(new ForwardDeceleration(localizerFunction, drivetrainFunction, velocity.get()));
                    result("naturalForwardDeceleration", forwardDeceleration);
                    result("naturalStrafeDeceleration", forwardDeceleration);

                    code(Language.JAVA,
                            "public static ForesightConfig foresightConfig = new ForesightConfig(\n" +
                                    "            c -> {\n" +
                                    "                c.naturalForwardDeceleration.set(" + forwardDeceleration + ");\n" +
                                    "                c.naturalStrafeDeceleration.set(" + forwardDeceleration + ");\n" +
                                    "            }\n" +
                                    "    );");
                    break;
                case 2:
                    List<Double> forwardTranslational = runOpMode(new ForwardTranslational(localizerFunction, drivetrainFunction));
                    double forwardTranslationalPrimary = forwardTranslational.get(0);
                    double forwardTranslationalSecondary = forwardTranslational.get(1);
                    double coast = forwardTranslational.get(2);
                    double brake = forwardTranslational.get(3);
                    result("forwardTranslational Primary kP", forwardTranslationalPrimary);
                    result("forwardTranslational Secondary kP", forwardTranslationalSecondary);
                    result("strafeTranslational Primary kP", forwardTranslationalPrimary);
                    result("strafeTranslational Secondary kP", forwardTranslationalSecondary);
                    result("coast kV", coast);
                    result("brake kV", brake);

                    code(Language.JAVA,
                                    "                Controller primaryTranslationalForward = Controller.proportional(" + forwardTranslationalPrimary + ");\n" +
                                    "                Controller secondaryTranslationalForward = Controller.proportional(" + forwardTranslationalSecondary + ");\n" +
                                    "                Controller primaryTranslationalLateral = Controller.proportional(" + forwardTranslationalPrimary + ");\n" +
                                    "                Controller secondaryTranslationalLateral = Controller.proportional(" + forwardTranslationalSecondary + ");\n" +
                                    "                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));\n" +
                                    "                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));\n" +
                                    "                c.coast.set(Controller.proportionalFeedforward(" + coast + "));\n" +
                                    "                c.brake.set(Controller.proportionalFeedforward(" + brake + "));\n");
                    break;
                case 3:
                    double heading = runOpMode(new HeadingTuner(localizerFunction, drivetrainFunction));
                    result("heading kP", heading);

                    code(Language.JAVA,
                                    "                c.headingFeedback.set(Controller.proportional(" + heading + "));\n");
                    break;
                case 4:
                    List<Double> headingBraking = runOpMode(new HeadingBraking(localizerFunction, drivetrainFunction));
                    double headingLinear = headingBraking.get(0);
                    double headingQuadratic = headingBraking.get(1);
                    result("headingBrakingLinearCoefficient", headingLinear);
                    result("headingBrakingQuadraticCoefficient", headingQuadratic);

                    code(Language.JAVA,
                                    "                c.headingBrakeCoefficients.set(Vector2D.cartesian(" + headingLinear + ", " + headingQuadratic + "));\n");

                    break;
                case 5:
                    Inputs distanceBrakingInput = inputs("Distance", "The distance to drive in inches for the Forward and Strafe Braking Identifiers");
                    Inputs.Field<Double> distanceBraking = distanceBrakingInput.d("Distance").withDefault(36.0);
                    awaitInputs(distanceBrakingInput);

                    List<Double> forwardBraking = runOpMode(new ForwardBraking(localizerFunction, drivetrainFunction, distanceBraking.get()));
                    double forwardLinear = forwardBraking.get(0);
                    double forwardQuadratic = forwardBraking.get(1);
                    result("forwardBrakingLinearCoefficient", forwardLinear);
                    result("forwardBrakingQuadraticCoefficient", forwardQuadratic);

                    code(Language.JAVA,
                                    "                c.linearBrakeCoefficients.set(Matrix.diag("+forwardLinear+", "+forwardLinear+"));\n" +
                                    "                c.quadraticBrakeCoefficients.set(Matrix.diag("+forwardQuadratic+", "+forwardQuadratic+"));\n");
                    break;
                default:
                    break;
            }
tunerID = 6;
        }while(tunerID < 6);
    }
}

class ForwardVelocity extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double distance;
    private final ArrayDeque<Double> velocities = new ArrayDeque<>();
    public static double RECORD_NUMBER = 10;

    public ForwardVelocity(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double distance) {
        super("Max Forward Velocity", "A tuner for finding the maximum achievable forward velocity. This will drive forward for " + distance + " inches and then likely drift past that position.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        boolean end = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1,0,0);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        waitForStart();

        while (!end) {
            localizer.update();
            if (Math.abs(localizer.pose().x()) > distance) {
                end = true;
                drivetrain.stop();
            } else {
                drivetrain.drive(power, true);
                double currentVelocity = Math.abs(localizer.twist().toVector2D().x());
                velocities.addLast(currentVelocity);
                velocities.removeFirst();
            }
        }

        drivetrain.stop();
        double average = 0;
        for (double velocity : velocities) {
                average += velocity;
        }
        average /= velocities.size();
        return average;
    }
}

class ForwardDeceleration extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;
    double velocity;

    private final ArrayList<Double> accelerations = new ArrayList<>();

    public ForwardDeceleration(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction, double velocity) {
        super("Forward Deceleration", "A tuner for finding the deceleration of the robot when moving forward. This will move forward until it reaches " + velocity + " inches per second.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.velocity = velocity;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        accelerations.clear();
        double previousVelocity = 0;
        long previousTimeNano = 0;
        boolean stopping = false;

        localizer.setPose(Pose.zero());
        localizer.update();

        DrivePowers power = new DrivePowers(1, 0, 0);
        waitForStart();

        drivetrain.drive(power, false);

        while (!stopping) {
            localizer.update();
            double currentVelocity = localizer.twist().toVector2D().x();
            if (currentVelocity > velocity) {
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                stopping = true;
                drivetrain.stop(false);
            }
        }

        boolean end = false;

        while (!end) {
            localizer.update();
            double currentVelocity = localizer.twist().toVector2D().x();
            long currentTimeNano = System.nanoTime();
            double dt = (currentTimeNano - previousTimeNano) / 1e9;

            if (dt > 0) {
                double acceleration = (currentVelocity - previousVelocity) / dt;
                accelerations.add(acceleration);
            }

            previousVelocity = currentVelocity;
            previousTimeNano = currentTimeNano;

            if (Math.abs(currentVelocity) <= 1) {
                end = true;
            }
        }

        drivetrain.stop(false);

        double average = 0;

        for (double acceleration : accelerations) {
            average += acceleration;
        }

        if (accelerations.isEmpty()) {
            return 0.0;
        }

        average /= accelerations.size();

        return Math.abs(average);
    }
}

class ForwardTranslational extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public static double ALPHA_LARGE = 10.2;
    public static double ALPHA_SMALL = 6.2;
    private final double VEL_AGGRESSIVENESS = 0.85;
    private final double POWER = 0.4;
    private final double RUNTIME = 1.2;
    private final int SAMPLES = 15;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    public ForwardTranslational(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Forward Translational", "A tuner for finding the Forward Translational kP coefficients using system identification. This will move around 12-24 inches in front of the robot and then stop.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        waitForStart();
        timer.reset();
        lastTime = timer.seconds();
        drivetrain.drive(new DrivePowers(POWER, 0.0, 0.0), false);

        while (!done && !isStopRequested()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            if (!done) {
                times.add(timer.seconds());

                double forwardVelocity = Math.abs(localizer.twist().toVector2D().x());
                vMax = Math.max(vMax, forwardVelocity / POWER);

                velocities.add(forwardVelocity);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                }
            }
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        //  kP_large, kP_small, coast kV, and brake kV (scaled by aggressiveness factor)
        return List.of(kP_large, kP_small, kV, kV * VEL_AGGRESSIVENESS);
    }

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}

class HeadingTuner extends TuningOpMode<Double> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;
    public static double ALPHA = 18.25;

    private double tau;
    private double K;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();

    public HeadingTuner(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading Tuner", "A tuner for finding the Heading Tuning Coefficients using system identification. This will spin the robot in place for a couple seconds.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected Double runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        times.clear();
        velocities.clear();
        boolean done = false;
        double vMax = 0;
        double lastTime = 0.0;

        waitForStart();
        timer.reset();
        lastTime = timer.seconds();
        drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);

        while (!done && !isStopRequested()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            times.add(timer.seconds());

            double turnVel = Math.abs(localizer.velocity().omega);
            vMax = Math.max(vMax, turnVel / POWER);

            velocities.add(turnVel);

            if (timer.seconds() >= RUNTIME) {
                done = true;
                systemIdentification();
                drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
            }
            else
                drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);
        return calculatekP(ALPHA);
    }

    private double calculatekP(double alpha) {
        double kV = 1 / K;
        double kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}

class HeadingBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    public static double TURN_TIME = 2;
    public static double MAX_BRAKE_TIME = 4; //seconds, the robot shouldn't take longer than this to brake
    public static double PAUSE_TIME = 0;
    public static int trials = 12;
    public static double maxPower = 1;
    public static double minPower = 0.2;
    public static double bias = 1.5; // how much it favors doing trials with higher powers

    private final ElapsedTime timer = new ElapsedTime();

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;

    private double startHeading;
    private double measuredVelocity;
    private double totalHeading;
    private double previousHeading;

    public HeadingBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction) {
        super("Heading Braking", "A tuner for Heading Braking Coefficients. The robot will turn and brake at various speed levels.", false);

        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        List<Double> coefficients = Collections.emptyList();

        double[] POWERS = biasedGradient(trials, maxPower, minPower, bias);
        int iteration = 0;

        waitForStart();
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        timer.reset();

        while (state != State.DONE && !isStopRequested()) {
            localizer.update();
            double currentHeading = localizer.pose().heading();
            totalHeading += Angle.normalizeSigned(currentHeading - previousHeading);
            previousHeading = currentHeading;

            double power = POWERS[POWERS.length-1-iteration];

            switch (state) {
                case DRIVE: {
                    if (timer.seconds() > TURN_TIME) {
                        startHeading = totalHeading;
                        measuredVelocity = Math.abs(localizer.velocity().omega);
                        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drivetrain.drive(new DrivePowers(0.0, 0.0, power), false);
                    break;
                }
                case BRAKE: {
                    if (Math.abs(localizer.velocity().omega) > 0.05 && timer.seconds() < MAX_BRAKE_TIME) {
                        drivetrain.drive(new DrivePowers(0, 0, 0.0), false);
                        break;
                    }

                    double endHeading = totalHeading;
                    double brakingDistance = Math.abs(endHeading - startHeading);

                    velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

                    iteration++;

                    if (iteration >= POWERS.length) {
                        double[] c = quadraticFit(velocityToBrakingDistance);
                        coefficients = List.of(c[0], c[1]);
                        state = State.DONE;
                    } else {
                        timer.reset();
                        state = State.PAUSE;
                    }
                    break;
                }
                case PAUSE:{
                    if (timer.seconds() < PAUSE_TIME) {
                        break;
                    }
                    timer.reset();
                    state = State.DRIVE;
                }
                case DONE: {}
            }
        }
        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);

        return coefficients;
    }

    private enum State {
        DRIVE,
        BRAKE,
        PAUSE,
        DONE
    }

    private static double[] biasedGradient(
            int count,
            double max,
            double min,
            double bias
    ) {
        if (count < 2) return new double[]{  max};

        double[] values = new double[count];

        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);

            double curved = 1 - Math.pow(t, bias);

            values[i] = min + curved * (max - min);
        }

        return values;
    }
}

class ForwardBraking extends TuningOpMode<List<Double>> {
    Function<HardwareMap, Localizer> localizerFunction;
    Function<HardwareMap, Drivetrain> drivetrainFunction;

    private double[] POWERS;
    public double MAX_BRAKE_TIME = 5.0;
    public int trials = 5;
    public double maxPower = 0.7;
    public double minPower = 0.3;
    public double bias = 1.5;
    public double distance;
    public double IDLE_SECONDS = 1;

    private final ElapsedTime timer = new ElapsedTime();
    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;
    private Vector2D startPosition;
    private double measuredVelocity;

    public ForwardBraking(Function<HardwareMap, Localizer> localizerFunction, Function<HardwareMap, Drivetrain> drivetrainFunction,
                          double distance) {
        super("Forward Braking", "A tuner for finding the Forward Braking Coefficients by driving forward and backward at various speeds. Please ensure that you have plenty of room at least " + distance + " inches ahead of the robot, but also tile space behind and laterally around the robot.", false);
        this.localizerFunction = localizerFunction;
        this.drivetrainFunction = drivetrainFunction;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        Localizer localizer = localizerFunction.apply(hardwareMap);
        Drivetrain drivetrain = drivetrainFunction.apply(hardwareMap);

        localizer.setPose(Pose.zero());
        localizer.update();

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        List<Double> coefficients = Collections.emptyList();

        waitForStart();
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        timer.reset();

        drivetrain.drive(new DrivePowers(maxPower,0,0), false);
        double nextX = distance;

        while (state != State.DONE && !isStopRequested()) {
            localizer.update();
            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

            switch (state) {
                case DRIVE: {
                    if ((direction > 0 && localizer.pose().x() >= nextX) || (direction < 0 && localizer.pose().x() <= nextX)) {
                        startPosition = localizer.pose().toVector2D();
                        measuredVelocity = localizer.velocity().toVector2D().magnitude();
                        brake(drivetrain, localizer);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drive(drivetrain, localizer);
                    break;
                }
                case BRAKE: {
                    if (localizer.velocity().toVector2D().magnitude() > 0.5 && timer.seconds() < MAX_BRAKE_TIME) {
                        brake(drivetrain, localizer);
                        break;
                    }

                    collectTrialData(localizer, drivetrain);
                    break;
                }
                case WAIT: {
                    drivetrain.stop();
                    if (timer.seconds() > IDLE_SECONDS) {
                        nextX = localizer.pose().x() + (direction * distance);
                        state = State.DRIVE;
                    }
                    break;
                }
                case DONE: {}
            }
        }

        if (state == State.DONE) {
            double[] c = quadraticFit(velocityToBrakingDistance);
            coefficients = List.of(c[0], c[1]);
        }

        return coefficients;
    }


    private void drive(Drivetrain drivetrain, Localizer localizer) {
        drivetrain.drive(new DrivePowers(power * direction, 0.0, 0), false);
    }

    private void brake(Drivetrain drivetrain, Localizer localizer) {
        drivetrain.drive(new DrivePowers(0, 0, 0), false);
    }

    private void collectTrialData(Localizer localizer, Drivetrain drivetrain) {
        Vector2D endPosition = localizer.pose().toVector2D();
        double brakingDistance = endPosition.minus(startPosition).magnitude();

        velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

        iteration++;

        if (iteration >= POWERS.length) {
            drivetrain.stop();
            state = State.DONE;
        } else {
            state = State.WAIT;
            timer.reset();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        WAIT,
        DONE
    }

    private static double[] biasedGradient(int count, double max, double min, double bias) {
        if (count < 2) return new double[]{max};
        double[] values = new double[count];
        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);
            double curved = 1 - Math.pow(t, bias);
            values[i] = min + curved * (max - min);
        }
        return values;
    }
}
