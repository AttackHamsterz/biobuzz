package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BlueCenterTunnelFarParkfar", group = "Auto: Blue")
public class BlueCenterFarParkfar extends BioBuzzAutoOpMode{
    public BlueCenterFarParkfar() {
        super(BLUE | CENTER_START | VIA_TUNNEL | VISIT_FAR_FLOWER | PARK_FAR);
    }
}
