package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    public static double kp;
    public static double ki;
    public static double kd;

    private double target;
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setTarget(double target) {
        this.target = target;
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

    public double calculate(double current) {
        double error = target - current;

        double dt = timer.seconds();
        if (dt < 0.0001) dt = 0.0001;

        // P term
        double p = kp * error;

        // I term
        integralSum += error * dt;
        double i = ki * integralSum;

        // D term
        double d = kd * ((error - lastError) / dt);

        lastError = error;
        timer.reset();

        return p + i + d;
    }
}