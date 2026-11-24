package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueOffCenterOutsideCloseFarParkfar", group = "Auto: Blue")
public class BlueOffCenterCloseFarParkfar extends BioBuzzAutoOpMode {
    public BlueOffCenterCloseFarParkfar() {
        super(BLUE | OFF_CENTER_START | VIA_OUTSIDE | VISIT_CLOSE_FLOWER | VISIT_FAR_FLOWER | PARK_FAR);
    }
}
