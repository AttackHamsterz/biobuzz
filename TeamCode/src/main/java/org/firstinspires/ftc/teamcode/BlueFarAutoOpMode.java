package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Blue Far", group = "Auto: Blue")
@Disabled
public class BlueFarAutoOpMode extends FarAutoOpMode{
    @Override public void init() {
        setup(COLOR.BLUE, POSITION.FAR, true);
        super.init();
    }
}
