package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Blue Close", group="Robot")
public class BioBuzzBlueOpMode extends BioBuzzOpMode {
    @Override public void init() {
        setup(COLOR.BLUE, POSITION.CLOSE, false);
        super.init();
    }
}
