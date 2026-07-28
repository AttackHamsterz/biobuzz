package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sorter extends RobotPart {
    private final RevColorSensorV3 sensor1;
    private final RevColorSensorV3 sensor2;
    private final CRServo sortServo;
    private final CRServo ejectServo;

    @Override
    public void loop() {

    }

    public Sorter(StandardSetupOpMode ssom) {
        this.ssom = ssom;
        sensor1 = ssom.hardwareMap.get(RevColorSensorV3.class, "sensor1"); // color sensor
        sensor2 = ssom.hardwareMap.get(RevColorSensorV3.class, "sensor1"); // color sensor
        sortServo = ssom.hardwareMap.get(CRServo.class, "sortServo"); //Servos
        ejectServo = ssom.hardwareMap.get(CRServo.class, "ejectServo");

        sensor1.setGain(20.0f);
        sensor2.setGain(20.0f);

        sortServo.setDirection(CRServo.Direction.FORWARD);
        ejectServo.setDirection(CRServo.Direction.FORWARD);

        sortServo.setPower(0);
        ejectServo.setPower(0);

    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if ((DEBUG & 32) != 0) {
            telemetry.addData("sensor1", sensor1.getLightDetected());
            telemetry.addData("sensor2", sensor2.getLightDetected());
        }
        boolean error = false;
        if (!sensor1.isLightOn()) {
            ssom.telemetry.addLine("Error:  sensor1 color light is not on!");
            error = true;
        }
        if (!sensor2.isLightOn()) {
            ssom.telemetry.addLine("Error: sensor2 color light is not on!");
            error = true;
        }
    }
}

