package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCenterTunnelFarParkfar", group = "Auto: Red")
public class RedCenterTunnelFarParkfar extends BioBuzzAutoOpMode{
    public RedCenterTunnelFarParkfar() {
        super(RED | CENTER_START | VIA_TUNNEL | VISIT_FAR_FLOWER | PARK_FAR);
    }
}