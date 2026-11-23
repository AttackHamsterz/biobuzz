package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueOffCenterBlockCloseFarParkfar", group = "Auto: Blue")
public class BlueOffCenterBlockCloseFarParkfar extends BioBuzzAutoOpMode {
    public BlueOffCenterBlockCloseFarParkfar() {
        super(
            BLUE |
            OFF_CENTER_START |
            BLOCK_FIRST_DROP |
            VISIT_CLOSE_FLOWER |
            VISIT_FAR_FLOWER |
            FAR_PARK);
    }
}
