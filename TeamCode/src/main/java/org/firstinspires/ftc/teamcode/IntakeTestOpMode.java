package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOp: Intake Test", group="Robot")
public class IntakeTestOpMode extends OpMode {

    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");
    }

    @Override
    public void loop() {
        intakeMotor.setPower(gamepad1.left_trigger);
    }
}
