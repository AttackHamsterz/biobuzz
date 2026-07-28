package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Motion extends RobotPart{
    public final CRServo centerServo;

    public Motion(StandardSetupOpMode ssom){
        this.ssom = ssom;
        centerServo = ssom.hardwareMap.get(CRServo.class, "testServo");
    }

    @Override
    public void loop() {
        centerServo.setPower(ssom.gamepad1.left_stick_x);
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }
}
