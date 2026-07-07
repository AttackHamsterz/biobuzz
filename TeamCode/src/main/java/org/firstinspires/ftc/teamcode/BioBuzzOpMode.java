package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Decode", group="Robot")
@Disabled
public class BioBuzzOpMode extends StandardSetupOpMode {
    @Override public void loop() {
        // Update telemetry
        if(RobotPart.DEBUG != 0) {
            super.partsList.forEach(part -> part.getKey().getTelemetry(telemetry));
            if (!super.partsList.isEmpty()) telemetry.update();
        }
    }
}
