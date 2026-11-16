package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedCenterTunnelFarParkFar", group = "Auto: Red")
public class RedCenterTunnelFarParkFar extends BioBuzzAutoOpMode {
    public RedCenterTunnelFarParkFar() {
        super(
            RED |
            CENTER_START |
            VIA_SAFE_TUNNEL |
            VISIT_FAR_FLOWER |
            FAR_PARK);
    }
}
