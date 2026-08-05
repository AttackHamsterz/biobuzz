package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;
import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sorter extends RobotPart {
    private final RevColorSensorV3 sensor1;
    private final RevColorSensorV3 sensor2;
    private final CRServo sortServo;
    private final CRServo ejectServo;
    private static final double MIN_DIST_CM = 3.0;


    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    public enum BallColor{
        None(0),
        Green(1),
        Purple(2);

        private final String color;
        private double distance;
        private final float[] hsv;
        private static final float PURPLE_HUE_MIN = 190;
        private static final float VALUE_MIN = 0.09f;

        BallColor(int id) {
            if(id < 1 || id > 3) id = 0;
            this.color = (id == 1) ? "Yellow" : (id == 2) ? "Blue" : (id == 3) ? "Red"
                    : "None";
            this.hsv = new float[3];
            this.distance = 0;
        }

        public void setHSV(float[] newHsv){
            if(newHsv.length > 3) {
                this.hsv[0] = newHsv[0];
                this.hsv[1] = newHsv[1];
                this.hsv[2] = newHsv[2];
                this.hsv[3] = newHsv[3];
            }
        }

        public void setDistance(double distance){
            this.distance = distance;
        }

        @NonNull
        public String toString() {
            return color;
        }

        public static BallColor fromSensor(RevColorSensorV3 sensor) {
            BallColor ballColor = BallColor.None;
            float[] hsv = new float[3];
            double distance = 0;

            // Sanity
            if(sensor != null){
                // Get distance (if too far, then no color)
                distance = sensor.getDistance(DistanceUnit.CM);

                if(distance <= MIN_DIST_CM) {
                    // Get normalized RGB (adjust gain at each competition)
                    NormalizedRGBA colors = sensor.getNormalizedColors();

                    // Convert to HSV
                    Color.RGBToHSV(
                            (int) (colors.red * 255),
                            (int) (colors.green * 255),
                            (int) (colors.blue * 255),
                            hsv
                    );

                    //Checking the brightness to see if it's high enough to be a color value.
                    if (hsv[2] >= VALUE_MIN) {
                        if (hsv[0] >= PURPLE_HUE_MIN)
                            ballColor = BallColor.Purple;
                        else
                            ballColor = BallColor.Green;
                    }
                    else
                        ballColor = BallColor.Purple;
                }
            }

            // Set hsv and distance and return
            ballColor.setDistance(distance);
            ballColor.setHSV(hsv);
            return ballColor;
        }
    }
    public Sorter(StandardSetupOpMode ssom) {
        this.ssom = ssom;
        sensor1 = ssom.hardwareMap.get(RevColorSensorV3.class, "sensor1"); // color sensor
        sensor2 = ssom.hardwareMap.get(RevColorSensorV3.class, "sensor2"); // color sensor
        sortServo = ssom.hardwareMap.get(CRServo.class, "sortServo"); //Servos
        ejectServo = ssom.hardwareMap.get(CRServo.class, "ejectServo");

        sensor1.setGain(20.0f);
        sensor2.setGain(20.0f);

        sortServo.setDirection(CRServo.Direction.FORWARD);
        ejectServo.setDirection(CRServo.Direction.FORWARD);

        sortServo.setPower(0);
        ejectServo.setPower(0);

    }

    @Override
    public void getTelemetry(Telemetry telemetry) {
        if ((DEBUG & 32) != 0) {
            telemetry.addData("sensor1", sensor1.getLightDetected());
            telemetry.addData("sensor2", sensor2.getLightDetected());
        }
        boolean error = false;
        if (!sensor1.isLightOn()) {
            ssom.telemetry.addLine("Error:  sensor1 color light is not on!");
            error = true;
        }
        if (!sensor2.isLightOn()) {
            ssom.telemetry.addLine("Error: sensor2 color light is not on!");
            error = true;
        }
        // check color
        telemetry.addData("sensor1Color", BallColor.fromSensor(sensor1).color);
    }
}

