package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightSwerveTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightSwerveTuner(
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                (hardwareMap) -> new HamSwerve(hardwareMap, Constants.driveConfig, new GearedCoaxialPod(hardwareMap, Constants.backLeft), new GearedCoaxialPod(hardwareMap, Constants.frontLeft), new GearedCoaxialPod(hardwareMap, Constants.backRight), new GearedCoaxialPod(hardwareMap, Constants.frontRight)));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new HamSwerve(hardwareMap, Constants.driveConfig, new GearedCoaxialPod(hardwareMap, Constants.backLeft), new GearedCoaxialPod(hardwareMap, Constants.frontLeft), new GearedCoaxialPod(hardwareMap, Constants.backRight), new GearedCoaxialPod(hardwareMap, Constants.frontRight)),
                (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }
}