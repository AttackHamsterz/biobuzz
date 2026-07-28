package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto", group = "Robot")
@Disabled

public class AutoOpMode extends StandardSetupOpMode {
    protected int pathState;
    protected Timer pathTimer;
    protected Timer opmodeTimer;

public void buildPaths() {}
public void autonomousPathUpdate() {}
public void setPathState (int pState) {
    int pathState = pState;
    pathTimer.resetTimer();
}
public void incrementPathState() {setPathState(pathState+1); }

@Override public void init() {
    super.init();
    pathTimer = new Timer();
    buildPaths();
}

@Override public void start() {
    setPathState(0);
    opmodeTimer.resetTimer();
}

}
