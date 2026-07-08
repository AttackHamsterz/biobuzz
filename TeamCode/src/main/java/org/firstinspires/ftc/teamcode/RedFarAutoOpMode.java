package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Red Far", group = "Auto: Red")
@Disabled
public class RedFarAutoOpMode extends FarAutoOpMode{
    @Override public void init() {
        setup(COLOR.RED, POSITION.FAR, true);
        super.init();
    }
}
