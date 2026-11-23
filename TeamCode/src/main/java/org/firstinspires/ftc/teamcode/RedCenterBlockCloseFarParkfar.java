package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCenterBlockCloseFarParkfar", group = "Auto: Red")
public class RedCenterBlockCloseFarParkfar extends BioBuzzAutoOpMode {
    public RedCenterBlockCloseFarParkfar() {
        super(
            RED |
            CENTER_START |
                    SHOOT_FIRST |
            BLOCK_FIRST_DROP |
            VISIT_CLOSE_FLOWER |
            VISIT_FAR_FLOWER |
                    SHOOT_SECOND |
            FAR_PARK);
    }
}
