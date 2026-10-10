package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart{
    private static final double ENOUGH_JOYSTICK = 0.1;
    private static final double MOTOR = 1;
    private final DcMotor pollenMotor;
    private static final double INTAKE_MOTOR_SPEED = 1.0;

    public Intake(StandardSetupOpMode ssom){

        this.ssom = ssom;
        pollenMotor = ssom.hardwareMap.get(DcMotor.class,"PollenIntake"); //need to define channel
        pollenMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    public void setPower(double power)
    {

    }


    @Override
    public void loop() {
        if (!ssom.ignoreGamepad)

        {
                pollenMotor.setPower(ssom.gamepad1.right_trigger);

                }





    }






    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 8) != 0){
            //telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        }

    }
}
