package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BallLifter extends RobotPart{

    private final DcMotor liftMotor;

    public BallLifter(StandardSetupOpMode ssom){
        this.ssom = ssom;
        liftMotor = ssom.hardwareMap.get(DcMotor.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        if(!ssom.ignoreGamepad) {
            if (ssom.gamepad1.a)
                liftMotor.setPower(1.0);
            else if(ssom.gamepad1.b)
                liftMotor.setPower(-1.0);
            else
                liftMotor.setPower(0.0);
        }
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 2) != 0){
            telemetry.addData("Lift Motor Power", liftMotor.getPower());
        }
    }
}
