package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lifter extends RobotPart{
    private static final int CLOSE_ENOUGH = 20;
    private static final int DOWN_TICKS = 0;
    private static final int UP_TICKS = 100;
    private static final double LIFT_POWER = 0.8;
    private static final double HOLD_POWER = 0.0;

    private final DcMotorEx liftMotor;
    private int targetPosition;

    public Lifter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        liftMotor = ssom.hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor.setTargetPositionTolerance(CLOSE_ENOUGH);
        targetPosition = DOWN_TICKS;
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
        if(!ssom.ignoreGamepad) {
            // Switch modes
            targetPosition = (ssom.launcher.isFlowerMode()) ? UP_TICKS : DOWN_TICKS;

            int currentPosition = liftMotor.getCurrentPosition();
            if ( Math.abs(targetPosition - currentPosition) > CLOSE_ENOUGH)
                liftMotor.setTargetPosition(targetPosition);
        }
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 32) != 0) {
            telemetry.addData("targetLiftPosition", targetPosition);
        }
    }
}