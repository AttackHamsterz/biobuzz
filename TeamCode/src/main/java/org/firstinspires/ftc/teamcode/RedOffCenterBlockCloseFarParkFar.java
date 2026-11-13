package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedOffCenterBlockCloseFarParkFar", group = "Auto: Red")
public class RedOffCenterBlockCloseFarParkFar extends BioBuzzAutoOpMode {
    public RedOffCenterBlockCloseFarParkFar() {
        super(
            RED |
            OFF_CENTER_START |
            BLOCK_FIRST_DROP |
            VISIT_CLOSE_FLOWER |
            VISIT_FAR_FLOWER |
            FAR_PARK);
    }
}
