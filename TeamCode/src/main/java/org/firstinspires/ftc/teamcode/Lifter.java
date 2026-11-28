package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lifter extends RobotPart{
    private static final int CLOSE_ENOUGH = 10;
    private static final int DOWN_TICKS = 0;
    private static final int UP_TICKS = 40;
    private static final double LIFT_POWER = 0.8;

    private final DcMotorEx liftMotor;
    private int targetPosition;

    public Lifter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        liftMotor = ssom.hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        targetPosition = DOWN_TICKS;
        liftMotor.setTargetPosition(targetPosition);

        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPositionTolerance(CLOSE_ENOUGH);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setPower(LIFT_POWER);
    }

    @Override
    public void init() {

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {
        // To lift or not to lift
        if (!ssom.ignoreGamepad) {
            targetPosition = (ssom.launcher.isFlowerMode()) ? UP_TICKS : DOWN_TICKS;

            // Simply update the target; RUN_TO_POSITION handles moving and holding automatically
            liftMotor.setTargetPosition(targetPosition);
            liftMotor.setPower(LIFT_POWER);
        }
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 32) != 0) {
            telemetry.addData("targetLiftPosition", targetPosition);
            telemetry.addData("currentLiftPosition", liftMotor.getCurrentPosition());
            telemetry.addData("isBusy", liftMotor.isBusy());
        }
    }
}