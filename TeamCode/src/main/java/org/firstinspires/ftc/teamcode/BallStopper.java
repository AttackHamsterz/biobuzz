package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import com.qualcomm.robotcore.hardware.Servo;

public class BallStopper extends RobotPart {


    public Servo ballServo = null;

    private static final double NECTAR_POLLEN_OPEN = 0.0;
    private static final double NECTAR_OPEN = 0.225;
    private static final double ALL_CLOSED = 0.5;
    private static final double POLLEN_OPEN = 0.78;

    private static final double POLLEN_NECTAR_OPEN = 1.00;




    public BallStopper(StandardSetupOpMode ssom) {
        this.ssom = ssom;

        ballServo = ssom.hardwareMap.get(Servo.class, "ball_stop");
        ballServo.setPosition(POSITION_CLOSED);
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
            if (this.ssom.gamepad1.right_bumper && this.ssom.gamepad1.left_bumper)
            {

                if  (ballServo.getPosition() < POSITION_CLOSED) {
                    ballServo.setPosition((NECTAR_CLOSED));// 0.0
                }
                else
                {
                    ballServo.setPosition((POLLEN_OPEN_1));// 1.00
                }

            }
            else if (this.ssom.gamepad1.right_bumper)
            {
                ballServo.setPosition(POLLEN_OPEN); // 0.78
            }
            else if (this.ssom.gamepad1.left_bumper)
            {
                ballServo.setPosition(NECTAR_OPEN); // 0.25

            }
            else {
                ballServo.setPosition((POSITION_CLOSED)); // 0.5
            }
          /*   if (this.ssom.gamepad1.right_trigger_pressed)
            {
                ballServo.setPosition(POLLEN_OPEN);
            }
             else {
                 ballServo.setPosition((POSITION_CLOSED));//Release the ball
             }
             if (this.ssom.gamepad1.left_trigger_pressed)
            {
                ballServo.setPosition(POLLEN_OPEN);
            }
            else {
                ballServo.setPosition(POSITION_CLOSED);
            }*/
        }
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }

    public void Stop()
    {

    }
}
