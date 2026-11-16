package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BlueCenterBlockFarParkFar", group = "Auto: Blue")
@Disabled
public class BlueCenterBlockFarParkFar extends BioBuzzAutoOpMode{
    public BlueCenterBlockFarParkFar() {
        super(
            BLUE |
            CENTER_START |
            BLOCK_FIRST_DROP |
            VISIT_FAR_FLOWER |
            FAR_PARK);
    }
}
