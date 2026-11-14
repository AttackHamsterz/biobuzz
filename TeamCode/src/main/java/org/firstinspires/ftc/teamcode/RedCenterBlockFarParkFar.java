package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RedCenterBlockFarParkFar", group = "Auto: Red")
public class RedCenterBlockFarParkFar extends BioBuzzAutoOpMode {
    public RedCenterBlockFarParkFar() {
        super(
            RED |
            CENTER_START |
            BLOCK_FIRST_DROP |
            VISIT_CLOSE_FLOWER |
            VISIT_FAR_FLOWER |
            FAR_PARK);
    }
}
