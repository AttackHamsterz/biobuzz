package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedOffCenterOutsideCloseFarParkfar", group = "Auto: Red")
public class RedOffCenterCloseFarParkfar extends BioBuzzAutoOpMode {
    public RedOffCenterCloseFarParkfar() {
        super(RED | OFF_CENTER_START | VIA_OUTSIDE | VISIT_CLOSE_FLOWER | VISIT_FAR_FLOWER | PARK_FAR);
    }
}
