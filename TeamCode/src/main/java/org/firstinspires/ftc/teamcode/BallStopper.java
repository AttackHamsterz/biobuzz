package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

public class BallStopper extends RobotPart {

    public Servo ballServo = null;

    private static final double NECTAR_POLLEN_OPEN = 0.0;
    private static final double NECTAR_OPEN = 0.25;
    private static final double ALL_CLOSED = 0.5;
    private static final double POLLEN_OPEN = 0.8;

    private static final double POLLEN_NECTAR_OPEN = 1.00;

    public BallStopper(StandardSetupOpMode ssom) {
        this.ssom = ssom;
        ballServo = ssom.hardwareMap.get(Servo.class, "ballStop");
        ballServo.setPosition(ALL_CLOSED);
    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (!ssom.ignoreGamepad)
        {
            // The drive can press both launch buttons or just one
            // The code will select the closest correct position for the cam and set it
            if (this.ssom.gamepad1.right_bumper && this.ssom.gamepad1.left_bumper)
            {
                if  (ballServo.getPosition() < ALL_CLOSED)
                    ballServo.setPosition((NECTAR_POLLEN_OPEN));
                else
                    ballServo.setPosition((POLLEN_NECTAR_OPEN));
            }
            else if (this.ssom.gamepad1.right_bumper)
                ballServo.setPosition(NECTAR_OPEN);
            else if (this.ssom.gamepad1.left_bumper)
                ballServo.setPosition(POLLEN_OPEN);
            else
                ballServo.setPosition((ALL_CLOSED));
        }
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if((DEBUG & 2) != 0){
            telemetry.addData("Ball Servo Position", ballServo.getPosition());
        }
    }

    public void allowPollen(){
        ballServo.setPosition(POLLEN_OPEN);
    }

    public void allowNectar(){
        ballServo.setPosition(NECTAR_OPEN);
    }

    public void allowAll(){
        ballServo.setPosition(POLLEN_NECTAR_OPEN);
    }

    public void stopAll(){
        ballServo.setPosition(ALL_CLOSED);
    }
}
