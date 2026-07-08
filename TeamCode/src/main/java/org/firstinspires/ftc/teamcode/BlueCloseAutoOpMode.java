package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Blue Close", group = "Auto: Blue")
@Disabled
public class BlueCloseAutoOpMode extends CloseAutoOpMode{
    @Override public void init() {
        setup(COLOR.BLUE, POSITION.CLOSE, true);
        super.init();
    }
}

