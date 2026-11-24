package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueOffCenterNoBlockOutsideCloseFarParkfar", group = "Auto: Blue")
public class BlueOffCenterNoBlockCloseFarParknear extends BioBuzzAutoOpMode {
    public BlueOffCenterNoBlockCloseFarParknear() {
        super(BLUE | OFF_CENTER_START | DISABLE_BLOCK | VIA_OUTSIDE | VISIT_CLOSE_FLOWER | VISIT_FAR_FLOWER);
    }
}
