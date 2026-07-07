package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PodTest extends RobotPart{

    private class Pod{
        public final CRServo servo;
        public final AnalogInput encoder;
        public boolean readFirstVoltage = true;
        public double voltage = 0;
        public double lastVoltage = 0;
        public double minVoltage = 0.019;
        public double maxVoltage = 3.234;
        public double angle = 0;
        public double zeroAngleFraction = 0;
        public double wrapCount = 0;

        public Pod(StandardSetupOpMode ssom, String prefix){
            servo = ssom.hardwareMap.get(CRServo.class, prefix + "Servo");
            encoder = ssom.hardwareMap.get(AnalogInput.class, prefix + "Encoder");
        }

        public void update() {
            // Get voltage and retain HW min/max
            voltage = encoder.getVoltage();
            if(voltage < minVoltage)
                minVoltage = voltage;
            if(voltage > maxVoltage)
                maxVoltage = voltage;

            if(readFirstVoltage) {
                lastVoltage = voltage;
                readFirstVoltage = false;
            }
            double delta = voltage - lastVoltage;
            if (delta < -maxVoltage/2.0) {
                wrapCount++;
            } else if (delta > maxVoltage/2.0) {
                wrapCount--;
            }
            lastVoltage = voltage;
            double voltageFraction = (voltage - minVoltage) / (maxVoltage - minVoltage);
            angle = (((((voltageFraction + wrapCount) / 7.0 - zeroAngleFraction) % 1.0) + 1.0) % 1.0) * 360.0;
        }

        public void getTelemetry(Telemetry telemetry, String prefix){
            telemetry.addData(prefix + " voltage", voltage);
            telemetry.addData(prefix + " wrapCount", wrapCount);
            telemetry.addData(prefix + " min Voltage", minVoltage);
            telemetry.addData(prefix + " max Voltage", maxVoltage);
            telemetry.addData(prefix + " angle", angle);
        }

        public void zero(){
            zeroAngleFraction = angle / 360.0;
        }
    };

    private Pod centerPod;
    private Pod leftPod;
    private Pod rightPod;
    private boolean pressed = false;

    public PodTest(StandardSetupOpMode ssom){
        this.ssom = ssom;
        centerPod = new Pod(ssom, "test");
        //centerPod = new Pod(ssom, "center");
        //leftPod = new Pod(ssom, "left");
        //rightPod = new Pod(ssom, "right");
    }

    @Override
    public void loop() {
        centerPod.servo.setPower(ssom.gamepad1.left_stick_x);
        //leftPod.servo.setPower(ssom.gamepad1.left_stick_x);
        //rightPod.servo.setPower(ssom.gamepad1.left_stick_x);

        if(ssom.gamepad1.x){
            if(!pressed){
                centerPod.zero();
                //leftPod.zero();
                //rightPod.zero();
            }
        }else{
            pressed = false;
        }

        centerPod.update();
        //leftPod.update();
        //rightPod.update();
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        centerPod.getTelemetry(telemetry, "center");
        //leftPod.getTelemetry(telemetry, "left");
        //rightPod.getTelemetry(telemetry, "right");
        telemetry.update();
    }
}
