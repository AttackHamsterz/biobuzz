package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

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
 * Per-loop CSV log of the launching mechanism (turret aim, camera, flywheel, gate, intake), to
 * diagnose aim jitter and track stutter. Written to /sdcard/FIRST/launchlogs on the Control Hub;
 * pull it with adb. Diagnostic only: it reads values and changes nothing.
 *
 * Launcher opens it in start(), writes a row at the end of each launcher loop (~20 ms, on the
 * launcher thread), and StandardSetupOpMode.stop() closes it. The file starts with "# key=value"
 * lines (opmode, battery, launcher/eye/intake/stopper constants). Read it with
 * pandas.read_csv(path, comment='#').
 *
 * Columns:
 *   t, dt            seconds since start, ms since previous row
 *   turretSetDeg     turret target angle (0-360), from the camera or driver 2
 *   turretActDeg     turret encoder angle (0-360)
 *   turretErrDeg     signed shortest-path error the turret controller saw
 *   turretPwr        power the turret servo is ACTUALLY getting (the last one set; inside the
 *                    deadband the code doesn't update it)
 *   turretInBand     1 if inside the 1 deg deadband
 *   turretTps        turret encoder ticks/s (75 ticks/deg). This encoder is on the pollenIntake
 *                    port, so it is also the speed the hub's pollen-track speed loop sees
 *   camCmdDeg        camera angle Eye assumes (last commanded servo position, not measured)
 *   camTgtDeg        Eye's new aim angle (turret follows -camTgtDeg in hive mode)
 *   camErrDeg        error Eye measured from the frame
 *   camMoved         1 if Eye moved the camera servo this Eye loop
 *   hasTarget        1 if Eye has an aim fiducial
 *   frameNew         1 if Eye's last result was a new Limelight frame (not a repeat)
 *   frameAgeMs       age of that frame when Eye used it (staleness + capture + pipeline latency)
 *   frames           count of new frames Eye has seen (changes when Eye processes a new one)
 *   aimX, aimY       aim fiducial position, robot space (m)
 *   flyTgtRpm        requested flywheel RPM; flyCmdRpm = after slew; flyActRpm = measured
 *   ready            launchReady() (the gate's open condition)
 *   gatePos          ball stopper servo position (commanded)
 *   pollenPwr, nectarPwr   commanded intake power
 *   pollenAmps, nectarAmps intake motor current (blank unless LOG_CURRENT)
 *   mode             0 = hive, 1 = flower
 *   g1LB..g2Stick    driver inputs (bumpers 0/1, triggers 0-1, g2Stick = turret override active)
 *   camFresh         1 if Eye acted on a fresh measurement (new frame, changed aim point)
 *   camCapDeg        camera angle Eye looked up for that frame's capture time (last fresh one)
 *   aimId            first center tag Eye aimed from (-1 = none)
 *   aimTags          2 if its partner center tag was averaged in (aim = midpoint), else 1
 *   centerSeen       how many of our center tags were in view
 */
public class LaunchLog {
    public static final String DIR = "/sdcard/FIRST/launchlogs";

    // Motor current is a separate hub read (~2-3 ms each), not part of the bulk read. Off by
    // default so logging doesn't slow the launcher loop.
    public static final boolean LOG_CURRENT = false;

    private static final String COLUMNS = "t,dt,"
            + "turretSetDeg,turretActDeg,turretErrDeg,turretPwr,turretInBand,turretTps,"
            + "camCmdDeg,camTgtDeg,camErrDeg,camMoved,hasTarget,frameNew,frameAgeMs,frames,aimX,aimY,"
            + "flyTgtRpm,flyCmdRpm,flyActRpm,ready,"
            + "gatePos,pollenPwr,nectarPwr,pollenAmps,nectarAmps,"
            + "mode,g1LB,g1RB,g1LT,g1RT,g2LT,g2RT,g2Stick,"
            + "camFresh,camCapDeg,aimId,aimTags,centerSeen";

    private final StandardSetupOpMode ssom;
    private BufferedWriter out;
    private final long startNs;
    private long lastNs;
    private final StringBuilder sb = new StringBuilder(512);
    public final String path;

    public LaunchLog(StandardSetupOpMode ssom, String name) {
        this.ssom = ssom;
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File dir = new File(DIR);
        //noinspection ResultOfMethodCallIgnored
        dir.mkdirs();
        File file = new File(dir, name + "_" + stamp + ".csv");
        path = file.getAbsolutePath();
        try {
            out = new BufferedWriter(new FileWriter(file), 1 << 16);
            writeSettings();
            out.write(COLUMNS);
            out.newLine();
        } catch (IOException e) {
            out = null;
        }
        startNs = System.nanoTime();
        lastNs = startNs;
    }

    /** One row. Called from the launcher thread only. */
    public synchronized void row() {
        if (out == null) return;
        long now = System.nanoTime();
        Launcher l = ssom.launcher;
        Eye e = ssom.eye;
        Intake in = ssom.intake;
        Gamepad g1 = ssom.gamepad1;
        Gamepad g2 = ssom.gamepad2;

        sb.setLength(0);
        sb.append(f((now - startNs) * 1e-9)).append(',').append(f((now - lastNs) * 1e-6));
        lastNs = now;

        // Turret
        d(l.getTurretSetpointDeg());
        d(l.getTurretActualDeg());
        d(l.getTurretErrorDeg());
        d(l.getTurretPower());
        b(l.isTurretInBand());
        d(l.getTurretTicksPerSec());

        // Camera
        if (e != null) {
            d(e.getCamAngleDeg());
            d(e.getTargetAngleDeg());
            d(e.getLastDegreeToTurn());
            b(e.getLastCamMoved());
            b(e.activeTarget());
            b(e.getLastFrameNew());
            d(e.getLastFrameAgeMs());
            sb.append(',').append(e.getFrameCount());
            Position aim = e.getAimResult();
            if (aim != null) {
                d(aim.x);
                d(aim.y);
            } else {
                sb.append(",,");
            }
        } else {
            sb.append(",,,,,,,,,");
        }

        // Flywheel
        d(l.getTargetRPM());
        d(l.getInterimRPM());
        d(l.getCurrentRPM());
        b(l.launchReady());

        // Gate and intake
        d(ssom.ballStopper != null ? ssom.ballStopper.ballServo.getPosition() : Double.NaN);
        if (in != null) {
            d(in.getPollenPower());
            d(in.getNectarPower());
            if (LOG_CURRENT) {
                d(in.getPollenAmps());
                d(in.getNectarAmps());
            } else {
                sb.append(",,");
            }
        } else {
            sb.append(",,,,");
        }

        // Mode and driver inputs
        sb.append(',').append(l.isFlowerMode() ? 1 : 0);
        b(g1.left_bumper);
        b(g1.right_bumper);
        d(g1.left_trigger);
        d(g1.right_trigger);
        d(g2.left_trigger);
        d(g2.right_trigger);
        b(Math.abs(g2.left_stick_y) > 0.05 || Math.abs(g2.left_stick_x) >= 0.05);

        // Camera fix diagnostics (appended so earlier columns keep their positions)
        if (e != null) {
            b(e.getLastFresh());
            d(e.getLastCamAtCaptureDeg());
            sb.append(',').append(e.getLastAimId());
            sb.append(',').append(e.getLastAimTags());
            sb.append(',').append(e.getLastCenterSeen());
        } else {
            sb.append(",,,,,");
        }

        try {
            out.write(sb.toString());
            out.newLine();
        } catch (IOException ex) {
            close();
        }
    }

    public synchronized void close() {
        if (out == null) return;
        try {
            out.flush();
            out.close();
        } catch (IOException ignored) {
        }
        out = null;
    }

    private void d(double v) {
        sb.append(',').append(f(v));
    }

    private void b(boolean v) {
        sb.append(',').append(v ? '1' : '0');
    }

    // ---- Settings header (once per run) ----

    private void writeSettings() throws IOException {
        List<String> lines = new ArrayList<>();
        lines.add("opmode=" + ssom.getClass().getSimpleName());
        lines.add("color=" + ssom.color);
        try {
            lines.add("battery.voltageAtStart="
                    + f(ssom.hardwareMap.voltageSensor.iterator().next().getVoltage()));
        } catch (RuntimeException ignored) {
        }
        lines.add("LaunchLog.LOG_CURRENT=" + LOG_CURRENT);
        constants(Launcher.class, lines);
        constants(Eye.class, lines);
        constants(Intake.class, lines);
        constants(BallStopper.class, lines);
        Collections.sort(lines);
        for (String line : lines) {
            out.write("# ");
            out.write(line);
            out.newLine();
        }
    }

    // Every static number/boolean declared in the class (private ones included): gains,
    // deadbands, powers, RPM limits.
    private static void constants(Class<?> cls, List<String> lines) {
        for (Field fld : cls.getDeclaredFields()) {
            if (!Modifier.isStatic(fld.getModifiers())) continue;
            Class<?> t = fld.getType();
            if (t != double.class && t != int.class && t != boolean.class) continue;
            try {
                fld.setAccessible(true);
                lines.add(cls.getSimpleName() + "." + fld.getName() + "=" + fld.get(null));
            } catch (Exception ignored) {
            }
        }
    }

    // 4 decimals without String.format (slow on Android).
    private static String f(double v) {
        if (Double.isNaN(v) || Double.isInfinite(v)) return "";
        return Double.toString(Math.round(v * 10000.0) / 10000.0);
    }
}
