package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp: Red Close", group="Robot")
public class BioBuzzRedOpMode extends BioBuzzOpMode {
    @Override public void init() {
        setup(COLOR.RED, POSITION.CLOSE, false);
        super.init();
    }
}
