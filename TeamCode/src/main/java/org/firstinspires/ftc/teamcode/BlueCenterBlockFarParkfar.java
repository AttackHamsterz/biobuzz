package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BlueCenterBlockFarParkfar", group = "Auto: Blue")
@Disabled
public class BlueCenterBlockFarParkfar extends BioBuzzAutoOpMode{
    public BlueCenterBlockFarParkfar() {
        super(
            BLUE |
            CENTER_START |
                    SHOOT_FIRST |
            BLOCK_FIRST_DROP |
            VISIT_FAR_FLOWER |
                    SHOOT_SECOND |
            FAR_PARK);
    }
}
