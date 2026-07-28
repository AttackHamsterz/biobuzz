package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PodTest extends RobotPart{

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

        public Pod(StandardSetupOpMode ssom, String prefix){
            // Hardware mapping
            motor = ssom.hardwareMap.get(DcMotorEx.class, prefix + "Motor");
            servo = ssom.hardwareMap.get(CRServo.class, prefix + "Servo");
            encoder = ssom.hardwareMap.get(AnalogInput.class, prefix + "Encoder");
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
            telemetry.addData(prefix + " count", encoderCount);
            telemetry.addData(prefix + " angle", angle);
        }

        public void zero(){
            zeroVoltage = encoder.getVoltage();
        }
    };

    private Pod testPod;
    private boolean pressed = false;

    public PodTest(StandardSetupOpMode ssom){
        this.ssom = ssom;
        testPod = new Pod(ssom, "test");
    }

    @Override
    public void loop() {
        if(ssom.gamepad1.b){
            if(!pressed){
                testPod.zero();
                pressed = true;
            }
        }else if(ssom.gamepad1.a){
            if(!pressed){
                testPod.init();
                pressed = true;
            }
        }else{
            pressed = false;
        }
        testPod.motor.setPower(ssom.gamepad1.left_stick_y);

        testPod.update();
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        testPod.getTelemetry(telemetry, "center");
        telemetry.update();
    }

    public void init(){
        //testPod.init();
    }
}
