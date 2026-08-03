package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name="TeleOp: Swerve Calibration", group="Robot")
public class SwerveCalibrationOpMode extends OpMode {

    private List<LynxModule> allHubs;
    private Pod flPod;
    private Pod frPod;
    private Pod blPod;
    private Pod brPod;
    private boolean pressed = false;

    /**
     * A pod has everything we need for swerve.  This module uses the analog
     * port to zero the wheel on a voltage (angle).  The wheel must be within
     * +/- 50 degrees to center correctly.  Once zeroed we switch to the digital
     * encoder tics from the motor input for the current angle of the wheel.
     */
    private class Pod{
        private static final double ELC_SCALE_FACTOR = 1.0 / (4000.0 * 7.0);
        public final DcMotorEx motor;
        public final CRServo servo;
        public final AnalogInput encoder;
        public double zeroVoltage;
        private double minVoltage;
        private double maxVoltage;
        public int encoderCount;
        public double startAngleFraction = 0;
        public double angle = 0;

        public Pod(HardwareMap hardwareMap, String prefix, double zeroVoltage, double minVoltage, double maxVoltage){
            // Hardware mapping
            motor = hardwareMap.get(DcMotorEx.class, prefix + "Motor");
            servo = hardwareMap.get(CRServo.class, prefix + "Servo");
            encoder = hardwareMap.get(AnalogInput.class, prefix + "Encoder");
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
            this.zeroVoltage = zeroVoltage;
            this.minVoltage = minVoltage;
            this.maxVoltage = maxVoltage;
        }

        public void init(){
            // Get current voltage
            double currentVoltage = encoder.getVoltage();

            // Determine how far away from our zero angle orientation voltage
            double errorVoltage = currentVoltage - zeroVoltage;
            errorVoltage = errorVoltage % 3.3;
            if (errorVoltage > 1.65)
                errorVoltage -= 3.3;
            else if (errorVoltage < -1.65)
                errorVoltage += 3.3;

            // Convert error into startAngle
            startAngleFraction = errorVoltage / (maxVoltage - minVoltage) / 7.0;

            // Reset the encoder now that we know the start angle
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void update() {
            // Encoder position after zeroing gives us our angle
            encoderCount = motor.getCurrentPosition();
            angle = ((((double)encoderCount * ELC_SCALE_FACTOR + startAngleFraction) + 1.0) % 1.0) * 360.0;

            // Watch for min and max voltages
            double voltage = encoder.getVoltage();
            if(voltage < minVoltage)
                minVoltage = voltage;
            if(voltage > maxVoltage)
                maxVoltage = voltage;
        }

        public void getTelemetry(Telemetry telemetry, String prefix){
            telemetry.addData(prefix + " zero", zeroVoltage);
            telemetry.addData(prefix + " min", minVoltage);
            telemetry.addData(prefix + " max", maxVoltage);
            telemetry.addData(prefix + " startAngle", startAngleFraction * 360.0);
            telemetry.addData(prefix + " angle", angle);
            telemetry.addData(prefix + " count", encoderCount);
        }

        public void zero(){
            zeroVoltage = encoder.getVoltage();
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
        flPod = new Pod(hardwareMap, "frontLeft", Constants.FRONT_LEFT_ZERO, Constants.FRONT_LEFT_MIN, Constants.FRONT_LEFT_MAX);
        frPod = new Pod(hardwareMap, "frontRight", Constants.FRONT_RIGHT_ZERO, Constants.FRONT_RIGHT_MIN, Constants.FRONT_RIGHT_MAX);
        blPod = new Pod(hardwareMap, "backLeft", Constants.BACK_LEFT_ZERO, Constants.BACK_LEFT_MIN, Constants.BACK_LEFT_MAX);
        brPod = new Pod(hardwareMap, "backRight", Constants.BACK_RIGHT_ZERO, Constants.BACK_RIGHT_MIN, Constants.BACK_RIGHT_MAX);
    }

    @Override
    public void start(){
        // Init the pods right before we start looping
        // Magnetic encoder should have settled
        flPod.init();
        frPod.init();
        blPod.init();
        brPod.init();
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
                flPod.init();
                frPod.init();
                blPod.init();
                brPod.init();
                pressed = true;
            }
        }else{
            pressed = false;
        }
        if(gamepad1.dpad_up){
            flPod.motor.setPower(gamepad1.left_stick_y);
            flPod.servo.setPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.dpad_right){
            frPod.motor.setPower(gamepad1.left_stick_y);
            frPod.servo.setPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.dpad_left) {
            blPod.motor.setPower(gamepad1.left_stick_y);
            blPod.servo.setPower(-gamepad1.left_stick_x);
        }
        else if(gamepad1.dpad_down) {
            brPod.motor.setPower(gamepad1.left_stick_y);
            brPod.servo.setPower(-gamepad1.left_stick_x);
        }
        else{
            flPod.servo.setPower(-gamepad1.left_stick_x);
            frPod.servo.setPower(-gamepad1.left_stick_x);
            blPod.servo.setPower(-gamepad1.left_stick_x);
            brPod.servo.setPower(-gamepad1.left_stick_x);
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
            telemetry.addLine("Press square for help");
            flPod.getTelemetry(telemetry, "frontLeft");
            frPod.getTelemetry(telemetry, "frontRight");
            blPod.getTelemetry(telemetry, "backLeft");
            brPod.getTelemetry(telemetry, "backRight");
        }
        telemetry.update();
    }
}
