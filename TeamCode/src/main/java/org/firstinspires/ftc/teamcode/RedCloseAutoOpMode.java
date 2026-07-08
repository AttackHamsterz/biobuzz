package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Red Close", group = "Auto: Red")
@Disabled
public class RedCloseAutoOpMode extends CloseAutoOpMode{
    @Override public void init() {
        setup(COLOR.RED, POSITION.CLOSE, true);
        super.init();
    }
}
