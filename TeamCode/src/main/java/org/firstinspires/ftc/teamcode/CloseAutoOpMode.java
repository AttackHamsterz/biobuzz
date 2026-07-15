package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto: Close", group = "Robot")
@Disabled
public class CloseAutoOpMode extends AutoOpMode {

    @Override public void init() {
        final double startPoseX = 0;
        final double startPoseY = 0;
        //add each pose number here so u dont have to tweak it in a bunch of cases

        // setup and build the paths
        super.init();
        //motion.follower.setStartingPose(startPose);
    }

    @Override
    public void buildPaths() {
        //put the pathing here :)
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                //something happens
                incrementPathState();
                break;

            case 1:
                //something else happens
                incrementPathState();
                break;
        }

    }

}
