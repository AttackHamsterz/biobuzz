package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightPowerAllocator;
import com.pedropathing.config.ConfigVar;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.drivetrains.SwervePod;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Per-loop CSV log of a path-following run, for tuning. Written to /sdcard/FIRST/pedrologs on the
 * Control Hub; pull it with adb. Diagnostic only: it reads values and changes nothing.
 *
 * Call {@link #row} once per loop right after follower.update(), and {@link #close} at the end.
 * Pod columns are in the pod encoder frame, after the >90 deg flip (see GearedCoaxialPod.move).
 *
 * The file starts with "# key=value" lines: a snapshot of every Constants number and the live
 * Foresight / swerve config, so each log records the settings it ran with. Read it with
 * pandas.read_csv(path, comment='#').
 */
public class RunLog {
    public static final String DIR = "/sdcard/FIRST/pedrologs";

    private final Follower follower;
    private BufferedWriter out;
    private final long startNs;
    private long lastNs;
    private final StringBuilder sb = new StringBuilder(1024);
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
            writeSettings();
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
            // Added 2026-11-06, after the pods so the older columns keep their positions.
            // vx,vy,omegaDeg: Pinpoint velocity, field frame (in/s, deg/s).
            // tvx,tvy: the same velocity in the robot frame (forward, left), what the brake
            //   coefficients multiply.
            // predX,predY,predHdeg: Foresight's predicted pose, pose + getBrakeDisplacement; the
            //   translational error is measured from here, not from the actual pose.
            // transVx,transVy / driveVx,driveVy / nffX,nffY: allocator vectors (translational
            //   correction, drive, normal feedforward), field frame, BEFORE clamping. Compare with
            //   cmdFwd/cmdStr (robot frame, after clamping and HamSwerve).
            // headPow, headFf: allocator heading power and heading feedforward, before clamping.
            // tgtVel, remain, braking: Foresight target velocity, remaining distance, braking flag.
            sb.append(",vx,vy,omegaDeg,tvx,tvy,predX,predY,predHdeg")
              .append(",transVx,transVy,driveVx,driveVy,nffX,nffY,headPow,headFf")
              .append(",tgtVel,remain,braking");
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
        Foresight fs = (a instanceof Foresight) ? (Foresight) a : null;
        if (fs != null) {
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
        } else {
            sb.append(",,,,");
        }

        // Localizer values are the Pinpoint's cached state from follower.update(): no hardware read.
        Velocity v = follower.velocity();
        Twist tw = follower.twist();
        sb.append(',').append(f(v.vx)).append(',').append(f(v.vy)).append(',').append(f(Math.toDegrees(v.omega)))
          .append(',').append(f(tw.vx)).append(',').append(f(tw.vy));

        if (fs != null) {
            // Same expression Foresight uses in calculatePath and calculateHold.
            Pose pred = p.plus(fs.getBrakeDisplacement(tw, p.heading()));
            sb.append(',').append(f(pred.x())).append(',').append(f(pred.y())).append(',').append(f(Math.toDegrees(pred.heading())));
            ForesightPowerAllocator al = fs.allocator;
            vec(al.getTranslationalVector());
            vec(al.getDriveVector());
            vec(al.getNormalFeedforwardVector());
            sb.append(',').append(f(al.getHeadingPower())).append(',').append(f(al.getHeadingFeedforward()))
              .append(',').append(f(fs.targetVelocity())).append(',').append(f(fs.remainingDistance()))
              .append(',').append(fs.isBraking() ? 1 : 0);
        } else {
            sb.append(",,,,,,,,,,,,,,");
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

    // The allocator's vectors are null until its first allocatePowers call.
    private void vec(Vector2D v) {
        if (v == null) {
            sb.append(",,");
        } else {
            sb.append(',').append(f(v.x())).append(',').append(f(v.y()));
        }
    }

    // ---- Settings header (once per run, so reflection's cost doesn't matter) ----

    private void writeSettings() throws IOException {
        List<String> lines = new ArrayList<>();
        // Every public static number/boolean in Constants: gains, caps, gate, pod zeros, turn PID.
        for (Field fld : Constants.class.getFields()) {
            int m = fld.getModifiers();
            if (!Modifier.isStatic(m)) continue;
            Class<?> t = fld.getType();
            if (t != double.class && t != int.class && t != boolean.class) continue;
            try {
                lines.add("Constants." + fld.getName() + "=" + fld.get(null));
            } catch (IllegalAccessException ignored) {
            }
        }
        // The live configs, read from the objects this follower actually runs with.
        Algorithm a = follower.algorithm();
        if (a instanceof Foresight) {
            configVars("foresight.", ((Foresight) a).config, lines);
        }
        configVars("swerve.", Constants.driveConfig, lines);
        configVars("pinpoint.", Constants.localizerConfig, lines);
        Collections.sort(lines);
        for (String line : lines) {
            out.write("# ");
            out.write(line);
            out.newLine();
        }
    }

    // Writes each public ConfigVar field of a Pedro config object. Controllers are skipped: their
    // gains aren't readable, which is why the gains live as named constants in Constants.
    private static void configVars(String prefix, Object cfg, List<String> lines) {
        for (Field fld : cfg.getClass().getFields()) {
            if (!ConfigVar.class.isAssignableFrom(fld.getType())) continue;
            try {
                Object val = ((ConfigVar<?>) fld.get(cfg)).get();
                if (val instanceof Controller) continue;
                String text = String.valueOf(val).replace('\n', ' ').replace('\r', ' ');
                lines.add(prefix + fld.getName() + "=" + text);
            } catch (Exception ignored) {
                // Unset required var or anything else odd: leave it out rather than fail the run.
            }
        }
    }

    // 4 decimals without String.format (slow on Android).
    private static String f(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "";
        return Double.toString(Math.round(v * 10000.0) / 10000.0);
    }
}
