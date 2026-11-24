package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Parent", group="A")
@Disabled
public class BioBuzzOpMode extends StandardSetupOpMode {
    @Override public void loop() {
        // Ensure ssom loop is called
        super.loop();

        // Update telemetry
        if(RobotPart.DEBUG != 0) {
            super.partsList.forEach(part -> part.getKey().getTelemetry(telemetry));
            if (!super.partsList.isEmpty()) telemetry.update();
        }
    }
}
