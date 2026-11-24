package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCenterOutsideFarParkfar", group = "Auto: Red")
public class RedCenterOutsideFarParkfar extends BioBuzzAutoOpMode{
    public RedCenterOutsideFarParkfar() {
        super(RED | CENTER_START | VIA_OUTSIDE | VISIT_FAR_FLOWER | PARK_FAR);
    }
}
