package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.GearedCoaxialPod;

import java.util.List;

@TeleOp(name="TeleOp: Swerve Calibration", group="Robot")
public class SwerveCalibrationOpMode extends OpMode {

    private List<LynxModule> allHubs;
    private Pod flPod;
    private Pod frPod;
    private Pod blPod;
    private Pod brPod;
    private Follower follower;
    private boolean pressed = false;

    /**
     * A pod has everything we need for swerve.  This module uses the analog
     * port to zero the wheel on a voltage (angle).  The wheel must be within
     * +/- 50 degrees to center correctly.  Once zeroed we switch to the digital
     * encoder tics from the motor input for the current angle of the wheel.
     */
    private class Pod{
        public final GearedCoaxialPod pod;
        public double zeroVoltage;
        private double minVoltage;
        private double maxVoltage;
        public int encoderCount;
        public double angle = 0;

        public Pod(GearedCoaxialPod pod, double zeroVoltage, double minVoltage, double maxVoltage){
            // Hardware mapping
            this.pod = pod;
            this.zeroVoltage = zeroVoltage;
            this.minVoltage = minVoltage;
            this.maxVoltage = maxVoltage;
        }

        public void update() {
            angle = pod.getAngle() * 180.0 / Math.PI;
            encoderCount = pod.getEncoderCount();

            // Watch for min and max voltages
            double voltage = pod.getVoltage();
            if(voltage < minVoltage)
                minVoltage = voltage;
            if(voltage > maxVoltage)
                maxVoltage = voltage;
        }

        public void getTelemetry(Telemetry telemetry, String prefix){
            telemetry.addData(prefix + " zero", zeroVoltage);
            telemetry.addData(prefix + " min", minVoltage);
            telemetry.addData(prefix + " max", maxVoltage);
            telemetry.addData(prefix + " voltage", pod.getVoltage());
            telemetry.addData(prefix + " startAngle", pod.getStartAngleFraction() * 360.0);
            telemetry.addData(prefix + " angle", angle);
            telemetry.addData(prefix + " count", encoderCount);
            telemetry.addData(prefix + " tpower", pod.targetPower);
            telemetry.addData(prefix + " tangle", pod.targetAngle);
        }

        public void zero(){
            zeroVoltage = pod.getVoltage();
        }
    }

    @Override
    public void init(){
        // 1. Get all hubs (Control Hub + Expansion Hub)
        allHubs = hardwareMap.getAll(LynxModule.class);

        // 2. Set to AUTO mode for multi-threading
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Setup swerve pods
        double ds = 0.1;
        GearedCoaxialPod flGPod = new GearedCoaxialPod(hardwareMap, Constants.frontLeft);
        GearedCoaxialPod frGPod = new GearedCoaxialPod(hardwareMap, Constants.frontRight);
        GearedCoaxialPod blGPod = new GearedCoaxialPod(hardwareMap, Constants.backLeft);
        GearedCoaxialPod brGPod = new GearedCoaxialPod(hardwareMap, Constants.backRight);

        flPod = new Pod(flGPod, Constants.FRONT_LEFT_ZERO, Constants.FRONT_LEFT_MIN+ds, Constants.FRONT_LEFT_MAX-ds);
        frPod = new Pod(frGPod, Constants.FRONT_RIGHT_ZERO, Constants.FRONT_RIGHT_MIN+ds, Constants.FRONT_RIGHT_MAX-ds);
        blPod = new Pod(blGPod, Constants.BACK_LEFT_ZERO, Constants.BACK_LEFT_MIN+ds, Constants.BACK_LEFT_MAX-ds);
        brPod = new Pod(brGPod, Constants.BACK_RIGHT_ZERO, Constants.BACK_RIGHT_MIN+ds, Constants.BACK_RIGHT_MAX-ds);

        Swerve swerve = new Swerve(hardwareMap, Constants.driveConfig,
                blGPod, flGPod, brGPod, frGPod);
        Foresight foresight = new Foresight(Constants.foresightConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        follower = new Follower(localizer, swerve, foresight);
    }

    @Override
    public void start(){
        // Init the pods right before we start looping
        // Magnetic encoder should have settled
        flPod.pod.init();
        frPod.pod.init();
        blPod.pod.init();
        brPod.pod.init();
    }

    @Override
    public void loop() {
        // Clear cache at the start of each loop cycle
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if(gamepad1.b){
            if(!pressed){
                flPod.zero();
                frPod.zero();
                blPod.zero();
                brPod.zero();
                pressed = true;
            }
        }else if(gamepad1.a){
            if(!pressed){
                flPod.pod.init();
                frPod.pod.init();
                blPod.pod.init();
                brPod.pod.init();
                pressed = true;
            }
        }else{
            pressed = false;
        }
        if(gamepad1.dpad_up){
            flPod.pod.setDrivePower(-gamepad1.left_stick_y);
            flPod.pod.setServoPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.dpad_right){
            frPod.pod.setDrivePower(-gamepad1.left_stick_y);
            frPod.pod.setServoPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.dpad_left) {
            blPod.pod.setDrivePower(-gamepad1.left_stick_y);
            blPod.pod.setServoPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.dpad_down) {
            brPod.pod.setDrivePower(-gamepad1.left_stick_y);
            brPod.pod.setServoPower(-gamepad1.left_stick_x);
        }
        else{
            double f = -gamepad1.left_stick_y;
            double s = -gamepad1.left_stick_x;
            double t = -gamepad1.right_stick_x;
            follower.manual(f, s, t);
            follower.update();
        }

        flPod.update();
        frPod.update();
        blPod.update();
        brPod.update();

        getTelemetry(telemetry);
    }

    public void getTelemetry(Telemetry telemetry) {
        if(gamepad1.x) {
            telemetry.addLine("Left stick left turns wheels counterclockwise from above");
            telemetry.addLine("Left stick forward spins wheels forward");
            telemetry.addLine("Flip directions in software if they do not behave");
            telemetry.addLine("Align wheels (gears face same direction +x).  Press circle for zeroVoltage");
            telemetry.addLine("Press x for new initAngle. Set zero voltages in software and recompile");
        }
        else {
            telemetry.addLine("Press square/x for help");
            flPod.getTelemetry(telemetry, "frontLeft");
            frPod.getTelemetry(telemetry, "frontRight");
            blPod.getTelemetry(telemetry, "backLeft");
            brPod.getTelemetry(telemetry, "backRight");
            telemetry.addData("lsx", gamepad1.left_stick_x);
            telemetry.addData("lsy", gamepad1.left_stick_y);
            telemetry.addData("rsx", gamepad1.right_stick_x);
        }
        telemetry.update();
    }
}
