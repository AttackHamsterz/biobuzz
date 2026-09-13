package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;

public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                (hardwareMap) -> new Swerve(hardwareMap, Constants.driveConfig, new GearedCoaxialPod(hardwareMap, Constants.backLeft), new GearedCoaxialPod(hardwareMap, Constants.frontLeft), new GearedCoaxialPod(hardwareMap, Constants.backRight), new GearedCoaxialPod(hardwareMap, Constants.frontRight)));
    }
}