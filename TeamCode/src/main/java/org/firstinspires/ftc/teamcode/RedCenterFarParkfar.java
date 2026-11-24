package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCenterTunnelFarParkfar", group = "Auto: Red")
public class RedCenterFarParkfar extends BioBuzzAutoOpMode{
    public RedCenterFarParkfar() {
        super(RED | CENTER_START | VIA_TUNNEL | VISIT_FAR_FLOWER | PARK_FAR);
    }
}