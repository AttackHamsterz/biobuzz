package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedOffCenterNoBlockOutsideCloseFarParkfar", group = "Auto: Red")
public class RedOffCenterNoBlockCloseFarParknear extends BioBuzzAutoOpMode {
    public RedOffCenterNoBlockCloseFarParknear() {
        super(RED | OFF_CENTER_START | DISABLE_BLOCK | VIA_OUTSIDE | VISIT_CLOSE_FLOWER | VISIT_FAR_FLOWER);
    }
}
