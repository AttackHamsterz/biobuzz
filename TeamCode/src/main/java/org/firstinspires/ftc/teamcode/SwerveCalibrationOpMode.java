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

import java.util.List;

@TeleOp(name="TeleOp: Swerve Calibration", group="Robot")
public class SwerveCalibrationOpMode extends OpMode {

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
        public double zeroVoltage = 0;
        public int encoderCount;
        public double startAngleFraction = 0;
        public double angle = 0;

        public Pod(HardwareMap hardwareMap, String prefix){
            // Hardware mapping
            motor = hardwareMap.get(DcMotorEx.class, prefix + "Motor");
            servo = hardwareMap.get(CRServo.class, prefix + "Servo");
            encoder = hardwareMap.get(AnalogInput.class, prefix + "Encoder");
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void init(){
            // Get current voltage
            double currentVoltage = encoder.getVoltage();

            // Determine how far away from our zero angle orientation voltage
            double error = currentVoltage - zeroVoltage;
            error = error % 3.3;
            if (error > 1.65)
                error -= 3.3;
            else if (error < -1.65)
                error += 3.3;

            // Convert error into startAngle
            startAngleFraction = error / 3.3;

            // Reset the encoder now that we know the start angle
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void update() {
            // Encoder position after zeroing gives us our angle
            encoderCount = motor.getCurrentPosition();
            angle = ((((double)encoderCount * ELC_SCALE_FACTOR + startAngleFraction) + 1.0) % 1.0) * 360.0;
        }

        public void getTelemetry(Telemetry telemetry, String prefix){
            telemetry.addData(prefix + " zeroVoltage", zeroVoltage);
            telemetry.addData(prefix + " startAngle", startAngleFraction * 360.0);
            telemetry.addData(prefix + " angle", angle);
            //telemetry.addData(prefix + " count", encoderCount);
        }

        public void zero(){
            zeroVoltage = encoder.getVoltage();
        }
    };

    private List<LynxModule> allHubs;
    private Pod lPod;
    private Pod rPod;
    private Pod bPod;
    private boolean pressed = false;

    @Override
    public void loop() {
        // Clear cache at the start of each loop cycle
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if(gamepad1.b){
            if(!pressed){
                lPod.zero();
                rPod.zero();
                bPod.zero();
                pressed = true;
            }
        }else if(gamepad1.a){
            if(!pressed){
                lPod.init();
                rPod.init();
                bPod.init();
                pressed = true;
            }
        }else{
            pressed = false;
        }
        lPod.motor.setPower(-gamepad1.left_stick_y);
        rPod.motor.setPower(-gamepad1.left_stick_y);
        bPod.motor.setPower(-gamepad1.left_stick_y);

        lPod.servo.setPower(-gamepad1.left_stick_x);
        rPod.servo.setPower(-gamepad1.left_stick_x);
        bPod.servo.setPower(-gamepad1.left_stick_x);

        lPod.update();
        rPod.update();
        bPod.update();
    }

    public void getTelemetry(Telemetry telemetry) {
        if(gamepad1.x) {
            telemetry.addLine("Left stick left turns wheels counterclockwise from above");
            telemetry.addLine("Left stick forward spins wheels forward");
            telemetry.addLine("Flip directions in software if they do not behave");
            telemetry.addLine("Align wheels (gears face same direction).  Press b for zeroVoltage");
            telemetry.addLine("Press a for new initAngle. Set zero voltages in software and recompile");
        }
        else {
            telemetry.addLine("Press x for help");
            lPod.getTelemetry(telemetry, "left");
            rPod.getTelemetry(telemetry, "right");
            bPod.getTelemetry(telemetry, "back");
        }
        telemetry.update();
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
        lPod = new Pod(hardwareMap, "left");
        rPod = new Pod(hardwareMap, "right");
        bPod = new Pod(hardwareMap, "back");
        lPod.init();
        rPod.init();
        bPod.init();
    }
}
