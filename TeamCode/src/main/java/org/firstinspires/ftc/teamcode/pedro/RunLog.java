package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.SwervePod;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Per-loop CSV log of a path-following run, for tuning. Written to /sdcard/FIRST/pedrologs on the
 * Control Hub; pull it with adb. Diagnostic only: it reads values and changes nothing.
 *
 * Call {@link #row} once per loop right after follower.update(), and {@link #close} at the end.
 * Pod columns are in the pod encoder frame, after the >90 deg flip (see GearedCoaxialPod.move).
 */
public class RunLog {
    public static final String DIR = "/sdcard/FIRST/pedrologs";

    private final Follower follower;
    private BufferedWriter out;
    private final long startNs;
    private long lastNs;
    private final StringBuilder sb = new StringBuilder(512);
    public final String path;

    public RunLog(Follower follower, String name) {
        this.follower = follower;
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File dir = new File(DIR);
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        File file = new File(dir, name + "_" + stamp + ".csv");
        path = file.getAbsolutePath();
        try {
            out = new BufferedWriter(new FileWriter(file), 1 << 16);
            sb.setLength(0);
            sb.append("t,dt,leg,x,y,hdeg,transErr,hdgErrDeg,cmdFwd,cmdStr,cmdRot,avgScale");
            HamSwerve swerve = swerve();
            if (swerve != null) {
                for (SwervePod pod : swerve.getPods()) {
                    String n = pod.name();
                    sb.append(',').append(n).append("_act")
                      .append(',').append(n).append("_des")
                      .append(',').append(n).append("_err")
                      .append(',').append(n).append("_turn")
                      .append(',').append(n).append("_drive");
                }
            }
            out.write(sb.toString());
            out.newLine();
        } catch (IOException e) {
            out = null;
        }
        startNs = System.nanoTime();
        lastNs = startNs;
    }

    private HamSwerve swerve() {
        return (follower.drivetrain instanceof HamSwerve) ? (HamSwerve) follower.drivetrain : null;
    }

    /** @param leg which leg of the test (e.g. 1 = out, 2 = back) so runs can be split apart. */
    public void row(int leg) {
        if (out == null) return;
        long now = System.nanoTime();
        sb.setLength(0);
        sb.append(f((now - startNs) * 1e-9)).append(',').append(f((now - lastNs) * 1e-9)).append(',').append(leg);
        lastNs = now;

        Pose p = follower.pose();
        sb.append(',').append(f(p.x())).append(',').append(f(p.y())).append(',').append(f(Math.toDegrees(p.heading())));

        Algorithm a = follower.algorithm();
        if (a instanceof Foresight) {
            Foresight fs = (Foresight) a;
            sb.append(',').append(f(fs.translationalError())).append(',').append(f(Math.toDegrees(fs.headingError())));
        } else {
            sb.append(",,");
        }

        HamSwerve s = swerve();
        if (s != null) {
            sb.append(',').append(f(s.getLastForward())).append(',').append(f(s.getLastStrafe()))
              .append(',').append(f(s.getLastRotation())).append(',').append(f(s.getLastAvgScaling()));
            List<SwervePod> pods = s.getPods();
            for (SwervePod pod : pods) {
                if (pod instanceof GearedCoaxialPod) {
                    GearedCoaxialPod g = (GearedCoaxialPod) pod;
                    sb.append(',').append(f(g.logActualDeg)).append(',').append(f(g.logDesiredDeg))
                      .append(',').append(f(g.logErrorDeg)).append(',').append(f(g.logTurnPower))
                      .append(',').append(f(g.logDrivePower));
                } else {
                    sb.append(",,,,,");
                }
            }
        }
        try {
            out.write(sb.toString());
            out.newLine();
        } catch (IOException e) {
            out = null;
        }
    }

    public void close() {
        if (out == null) return;
        try {
            out.flush();
            out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }

    // 4 decimals without String.format (slow on Android).
    private static String f(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "";
        return Double.toString(Math.round(v * 10000.0) / 10000.0);
    }
}
