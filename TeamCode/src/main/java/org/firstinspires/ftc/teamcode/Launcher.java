package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.ArrayList;

import com.pedropathing.controllers.Controller;
import com.pedropathing.utils.Angle;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher extends RobotPart{
    private static final double TPR = 28.0;
    private static final double MIN_RPM = 0.0;
    private static final double MAX_RPM = 6000.0;
    private static final double CLOSE_ENOUGH_RPM = 200;
    private static final double MIN_LAUNCH_VALUE_RPM = 2100;
    private static final double TPS_TO_RPM = 60.0 / TPR;
    private static final double RPM_TO_TPS = TPR / 60.0;
    private double MIN_THRESHOLD_DISTANCE = 0.5;
    private double MAX_THRESHOLD_DISTANCE = 2.75;

    private static final double kP = 30.0;//50.0;
    private static final double kI = 0.0;//1.0;
    private static final double kD = 200.0;//0.0;
    private static final double kF = 32767.0 / (MAX_RPM * RPM_TO_TPS);

    // Slew rate: max RPM change per loop tick (~100ms).
    // 1500 RPM/tick means 0→4000 RPM ramps over ~270ms (roughly 3 ticks).
    // This limits inrush current while keeping spin-up fast enough for gameplay.
    private static final double SLEW_UP_RPM_PER_TICK = 1500.0;

    private final DcMotorEx launchMotor;
    private volatile double targetVelocityRPM;  // What callers request
    private double interimVelocityRPM;           // Slewed value actually sent to motor
    private double currentVelocityRPM;
    private boolean isLaunchable;
    private TargetMode target;

    private ArrayList<RPMEntry> distanceRPM;
    private ArrayList<RPMEntry> velocityRPM;

    // FOR AUDREY
    // These variables capture turret rotation
    private static final double TWO_PI = Math.PI * 2.0;
    private static final double CLOSE_ENOUGH_RADIANS = 0.008726646; //  Half a degree seems close enough
    private static final double ENCODER_TICS_PER_REV = 4000.0;
    private static final double ENCODER_GEAR_RATIO = 6.75; // Small pulley 20 teeth, large pulley 135 teeth
    private static final double ELC_SCALE_FACTOR = 1.0 / (ENCODER_TICS_PER_REV * ENCODER_GEAR_RATIO);
    private static final double TURN_P = 0.9;
    private static final double TURN_I = 0.0;
    private static final double TURN_D = 0.085;
    private static final double TURN_F = 0.15;
    private final CRServo launchServo;
    private final Controller launchServoController; // PID uses radians (tune PIDF for radian error)
    private double launchAngleRadians;

    // Enumerations and definitions
    public enum TargetMode {
        HIVE(0),    // Shoot at the hive
        FLOWER(1),       // Shoot at the flower
        NONE(3);            // No mode, do nothing

        private final int value;
        TargetMode(int value){
            this.value = value;
        }
    }
    private class RPMEntry{
        public double value;
        public double rpm;
        public RPMEntry(double value, double rpm){
            this.value = value;
            this.rpm = rpm;
        }
    };

    // RPM calculations
    private double currentRPM;

    public Launcher(StandardSetupOpMode ssom){
        this.ssom = ssom;
        target = TargetMode.HIVE;
        launchMotor = ssom.hardwareMap.get(DcMotorEx.class, "launchMotor");
        launchServo = ssom.hardwareMap.get(CRServo.class, "launchServo");

        //Default to launchable until distance is outside the range.
        isLaunchable = true;

        // Setup PID and motor
        launchMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        launchMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Setup launch servo and digital encoder
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchServoController = Controller.pid(TURN_P, TURN_I, TURN_D).plus(Controller.proportionalFeedforward(TURN_F));
        launchAngleRadians = 0;

        // Initial Motor speeds
        targetVelocityRPM = 0;
        interimVelocityRPM = 0;
        currentVelocityRPM = 0;

        velocityRPM = new ArrayList<>(List.of(
                new RPMEntry(-50,-1500),
                new RPMEntry(-40,-1200),
                new RPMEntry(-20,-500),
                new RPMEntry(0,0),
                new RPMEntry(20,500),
                new RPMEntry(40,1200),
                new RPMEntry(50,1500)
        ));

        // Table of distances in meters to RPM
        distanceRPM = new ArrayList<>(List.of(
                new RPMEntry(0.0, 0.0),
                new RPMEntry(0.25, 1500.0),
                new RPMEntry(0.5, 2400.0),
                new RPMEntry(1.0, 2450.0),
                new RPMEntry(1.25, 2600.0),
                new RPMEntry(1.5, 2800.0),
                new RPMEntry(1.75, 3000.0),
                new RPMEntry(2.00, 3200.0),
                new RPMEntry(2.25, 3400.0),
                new RPMEntry(2.5, 3700.0),
                new RPMEntry(2.75, 4000.0)
        ));
    }

    public double getLaunchAngleDegrees(){
        return Math.toDegrees(getLaunchAngleRadians());
    }
    public double getLaunchAngleRadians(){
        return ((((double)-launchMotor.getCurrentPosition() * ELC_SCALE_FACTOR) + 1.0) % 1.0) * TWO_PI;
    }
    public void setLaunchAngleDegrees(double launchAngleDegrees){
        launchAngleRadians = Math.toRadians(launchAngleDegrees);
    }
    public void setLaunchAngleRadians(double launchAngleRadians) {
        this.launchAngleRadians = launchAngleRadians;
    }
    private void updateLaunchAngle(){
        // Convert hardware angle to radians and normalize
        double actualRad = Angle.normalize(getLaunchAngleRadians());
        double desiredRad = Angle.normalize(launchAngleRadians);

        // Shortest-path error in radians (signed)
        double mag = Angle.smallestDifference(actualRad, desiredRad);
        if (mag < CLOSE_ENOUGH_RADIANS) return; // Close enough, do nothing!
        double dir = Angle.turnDirection(actualRad, desiredRad);
        double errorRad = mag * dir;

        // Setpoint close to current so PID follows shortest path
        double turnPower = Utils.clamp(
                    launchServoController.calculate(0, errorRad),
                    -1.0,
                    1.0);
        launchServo.setPower(turnPower);
    }

    public void setTargetMode(TargetMode targetMode){
        target = targetMode;
    }

    public void setRPMFromDistance(double distance, double extraRPM){
        setRPMFromDistance(distance, extraRPM, 0);
    }

    public void setRPMFromDistance(double distance, double extraRPM, double V) {
        double rpm = 0;
        double vxrpm = 0;

        if (target == TargetMode.HIVE && (distance <= MIN_THRESHOLD_DISTANCE || distance >= MAX_THRESHOLD_DISTANCE)) {
            isLaunchable = false;
        } else {

            for (int i = 0; i < velocityRPM.size() - 1; i++) {
                if (V < velocityRPM.get(0).value)
                    vxrpm = velocityRPM.get(0).rpm;
                else if (V >= velocityRPM.get(velocityRPM.size() - 1).value)
                    vxrpm = velocityRPM.get(velocityRPM.size() - 1).rpm;
                else if (velocityRPM.get(i).value < V && V < velocityRPM.get(i + 1).value) {
                    double deltaX = velocityRPM.get(i + 1).value - velocityRPM.get(i).value;
                    double deltaRPM = velocityRPM.get(i + 1).rpm - velocityRPM.get(i).rpm;
                    vxrpm = velocityRPM.get(i).rpm + deltaRPM * ((V - velocityRPM.get(i).value) / deltaX);
                }
            }

            for (int i = 0; i < distanceRPM.size() - 1; i++) {
                if (distance < distanceRPM.get(0).value)
                    rpm = distanceRPM.get(0).rpm;
                else if (distance >= distanceRPM.get(distanceRPM.size() - 1).value)
                    rpm = distanceRPM.get(distanceRPM.size() - 1).rpm;
                else if (distanceRPM.get(i).value < distance && distance < distanceRPM.get(i + 1).value) {
                    double deltaDistance = distanceRPM.get(i + 1).value - distanceRPM.get(i).value;
                    double deltaRPM = distanceRPM.get(i + 1).rpm - distanceRPM.get(i).rpm;
                    rpm = distanceRPM.get(i).rpm + deltaRPM * ((distance - distanceRPM.get(i).value) / deltaDistance);
                }
            }
            setVelocityRPM(rpm + extraRPM + vxrpm);
        }
    }

    public void setVelocityRPM(double velocityRPM) {
        targetVelocityRPM = Range.clip(velocityRPM, MIN_RPM, MAX_RPM);
    }

    public boolean launchReady(){
        // Check target (not slewed) RPM so we don't report ready mid-ramp
        return targetVelocityRPM > MIN_LAUNCH_VALUE_RPM && Math.abs(targetVelocityRPM - currentVelocityRPM) < CLOSE_ENOUGH_RPM;
    }

    @Override
    public void init() {

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        // First step, make sure the launch servo is turned to the correct angle
        if(!ssom.ignoreGamepad) {
            if(ssom.gamepad1.dpad_up)
                launchAngleRadians = 0;
            else if(ssom.gamepad1.dpad_left)
                launchAngleRadians = Math.PI / 2.0;
            else if(ssom.gamepad1.dpad_down)
                launchAngleRadians = Math.PI;
            else if(ssom.gamepad1.dpad_right)
                launchAngleRadians = 3.0 * Math.PI / 2.0;
        }
        updateLaunchAngle();

        // Second step, make sure the launch motor is spinning at the right speed
        if(!ssom.ignoreGamepad){
            launchMotor.setPower(ssom.gamepad1.left_trigger);
        }
/*
            // Slew interimVelocityRPM toward targetVelocityRPM
            // Ramp UP gradually to limit inrush current; ramp DOWN instantly
            if (interimVelocityRPM < targetVelocityRPM) {
                interimVelocityRPM = Math.min(targetVelocityRPM, interimVelocityRPM + SLEW_UP_RPM_PER_TICK);
            } else {
                interimVelocityRPM = targetVelocityRPM;
            }

            // Send slewed velocity to motor
            launchMotor.setVelocity(interimVelocityRPM * RPM_TO_TPS);
            currentVelocityRPM = launchMotor.getVelocity() * TPS_TO_RPM;


        // Stop everything
        launchMotor.setVelocity(0);
        targetVelocityRPM = 0;
        interimVelocityRPM = 0;
        currentVelocityRPM = 0;
*/
    }


    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 2) != 0) {
            telemetry.addData("launchAngle", getLaunchAngleDegrees());
            telemetry.addData("launchTargetVelocityRPM", targetVelocityRPM);
            telemetry.addData("launchInterimVelocityRPM", interimVelocityRPM);
            telemetry.addData("launchCurrentVelocityRPM", currentVelocityRPM);
            telemetry.addData("Launchablility", isLaunchable ? "ITS A GO GO TIME" : "Please do not launch :(");
            telemetry.addData("LaunchMode",target);
        }
    }
}