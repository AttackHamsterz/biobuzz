package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends RobotPart {
    private final CRServo sortServo;
    private final CRServo ejectServo;

    public Elevator(StandardSetupOpMode ssom) {
        this.ssom = ssom;
        sortServo = ssom.hardwareMap.get(CRServo.class, "sortServo"); //Servos
        ejectServo = ssom.hardwareMap.get(CRServo.class, "ejectServo");

        sortServo.setDirection(CRServo.Direction.FORWARD);
        ejectServo.setDirection(CRServo.Direction.FORWARD);

        sortServo.setPower(0);
        ejectServo.setPower(0);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }
}

