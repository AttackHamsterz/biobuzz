package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends RobotPart{
    private static final double POLLEN_LAUNCH_POWER = 0.3;
    private static final double NECTAR_LAUNCH_POWER = 0.5;
    private static final double ENOUGH_JOYSTICK = 0.1;
    private static final double MOTOR = 1;
    private final DcMotor pollenMotor;
    private final DcMotor nectarMotor;
    private static final double INTAKE_MOTOR_SPEED = 1.0;

    public Intake(StandardSetupOpMode ssom){
        this.ssom = ssom;
        pollenMotor = ssom.hardwareMap.get(DcMotor.class,"pollenIntake"); //need to define channel
        pollenMotor.setDirection(DcMotor.Direction.REVERSE);
        nectarMotor = ssom.hardwareMap.get(DcMotor.class,"nectarIntake"); //need to define channel
        nectarMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    public void setPower(double power){
        nectarMotor.setPower(Range.clip(power, -1.0, 1.0));
        pollenMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void setNectarPower(double power){
        nectarMotor.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void setPollenPower(double power){
        pollenMotor.setPower(Range.clip(power,-1.0,1.0));
    }

    public void stop(){
        pollenMotor.setPower(0);
        nectarMotor.setPower(0);
    }

    @Override
    public void loop() {
        if(!ssom.ignoreGamepad){
            // The drive intakes with the left and right trigger while
            // driving.  To fire they hold down the left and right bumpers.
            if(ssom.gamepad1.left_bumper)
                pollenMotor.setPower(POLLEN_LAUNCH_POWER);
            else
                pollenMotor.setPower(ssom.gamepad1.left_trigger);
            if(ssom.gamepad1.right_bumper)
                nectarMotor.setPower(NECTAR_LAUNCH_POWER);
            else
                nectarMotor.setPower(ssom.gamepad1.right_trigger);
        }
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 8) != 0){
            telemetry.addData("Nectar Motor Power", nectarMotor.getPower());
            telemetry.addData("Pollen motor power", pollenMotor.getPower());
        }
    }
}
