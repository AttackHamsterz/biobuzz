package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret extends RobotPart {
    private double rotation = 0;

    @Override
    public void init() {

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() {

    }

    public double getRotation() {
        //get current location of turret
        return rotation;
    }

    public void setRotation(double degrees) {
        rotation = degrees;
    }

    @Override
    public void getTelemetry(Telemetry telemetry) {

    }
}
