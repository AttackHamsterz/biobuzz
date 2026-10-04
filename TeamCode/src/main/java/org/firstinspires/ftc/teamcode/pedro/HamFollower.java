package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.localization.Localizer;

import java.util.function.Consumer;

public class HamFollower extends Follower {
    private boolean zero = false;

    public HamFollower(Localizer localizer, Drivetrain drivetrain, Algorithm algorithm) {
        super(localizer, drivetrain, algorithm);
    }

    public void zero(){
        zero = true;
    }

    @Override
    public void update(double deltaTime) {
        if(zero) {
            ((HamSwerve)drivetrain).zero();
        }
        else{
            super.update(deltaTime);
        }
    }
}
