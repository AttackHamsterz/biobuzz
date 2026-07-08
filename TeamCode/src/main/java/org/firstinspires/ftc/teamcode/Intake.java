package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart{
    private static final double ENOUGH_JOYSTICK = 0.1;
    private static final double MOTOR = 1;
    private final DcMotor intakeMotor;
    private static final double INTAKE_MOTOR_SPEED = 1.0;

    public Intake(StandardSetupOpMode ssom){

        this.ssom = ssom;
        intakeMotor = ssom.hardwareMap.get(DcMotor.class,"intakeMotor"); //need to define channel

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

    }
    @Override
    public void loop() {
        if(!ssom.ignoreGamepad){
            if (ssom.gamepad1.a)
                intakeMotor.setPower(INTAKE_MOTOR_SPEED);
            else if(ssom.gamepad1.b)
                intakeMotor.setPower(-INTAKE_MOTOR_SPEED);
            else
                intakeMotor.setPower(0.0);
        }

    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 8) != 0){
            telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        }

    }
}
