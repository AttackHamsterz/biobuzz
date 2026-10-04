package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


import com.qualcomm.robotcore.hardware.Servo;

public class BallStopper extends RobotPart {


    public Servo ballServo = null;

    private static final double POSITION_CLOSED = 0.5;
    private static final double POLLEN_OPEN = 1.0;
    private static final double NECTAR_OPEN = 0.0;



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
            if (this.ssom.gamepad1.right_bumper)
            {
                ballServo.setPosition((NECTAR_OPEN));//Release the ball

            }

            else {
                ballServo.setPosition((POSITION_CLOSED));//Release the ball

            }
             if (this.ssom.gamepad1.left_bumper)
            {
                ballServo.setPosition(POLLEN_OPEN);
            }
             else {
                 ballServo.setPosition((POSITION_CLOSED));//Release the ball
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
