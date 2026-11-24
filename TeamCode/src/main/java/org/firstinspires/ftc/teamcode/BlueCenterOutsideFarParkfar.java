package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BlueCenterOutsideFarParkfar", group = "Auto: Blue")
public class BlueCenterOutsideFarParkfar extends BioBuzzAutoOpMode{
    public BlueCenterOutsideFarParkfar() {
        super(BLUE | CENTER_START | VIA_OUTSIDE | VISIT_FAR_FLOWER | PARK_FAR);
    }
}
