package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.Map;

/**
 * Wraps the Pinpoint localizer and drops readings the robot can't physically produce, keeping the
 * last good state instead:
 *  - all zeros (pose and velocity exactly 0) while away from the origin. 2026-11-08: the Line Test
 *    logged exact (0,0,0) poses for 1-3 loops mid-path; one made atParametricEnd() fire with 27 in
 *    left. A real reading is never exactly 0.0 on all six values while moving.
 *  - a jump bigger than the last good speed could cover since the last good read (x2, plus a
 *    margin). 2026-11-08: a Curve Test read had x jump 43.3 -> 2.7 for one loop with y, heading
 *    and velocity unchanged (a 29 in "path error").
 * Pedro's pose()/velocity()/twist() defaults all read state(), so overriding state() covers them.
 */
public class GuardedLocalizer implements Localizer {
    // Only treat all-zero as bad if the last good pose was farther than this from the origin.
    private static final double NEAR_ORIGIN_IN = 0.5;
    // Safety valve: accept zeros after this many in a row (a real Pinpoint reset, not a glitch).
    private static final int MAX_CONSECUTIVE_REJECTS = 10;
    // Jump check: allowed step = 2 x (last good speed x time since last good read) + this margin.
    // At 53 in/s and a 30 ms loop that is ~5 in; real steps are ~1.6 in.
    private static final double JUMP_MARGIN_IN = 2.0;
    private static final double JUMP_MARGIN_RAD = Math.toRadians(20);

    private final Localizer inner;
    private MotionState last;
    private long lastGoodNs = System.nanoTime();
    private int consecutive = 0;
    public int rejectedReads = 0; // total for the run; RunLog writes it as badReads

    public GuardedLocalizer(Localizer inner) {
        this.inner = inner;
        this.last = inner.state();
    }

    @Override
    public void update() {
        inner.update();
        MotionState s = inner.state();
        long now = System.nanoTime();
        if (isBad(s, (now - lastGoodNs) * 1e-9) && consecutive < MAX_CONSECUTIVE_REJECTS) {
            consecutive++;
            rejectedReads++;
            return; // keep the last good state
        }
        consecutive = 0;
        last = s;
        lastGoodNs = now;
    }

    private boolean isBad(MotionState s, double dtSinceGood) {
        if (last == null) return false;
        Pose p = s.pose();
        Velocity v = s.velocity();
        Pose lp = last.pose();
        Velocity lv = last.velocity();

        boolean allZero = p.x() == 0 && p.y() == 0 && p.heading() == 0
                && v.vx == 0 && v.vy == 0 && v.omega == 0;
        if (allZero && Math.hypot(lp.x(), lp.y()) > NEAR_ORIGIN_IN) return true;

        double step = Math.hypot(p.x() - lp.x(), p.y() - lp.y());
        double maxStep = 2 * Math.hypot(lv.vx, lv.vy) * dtSinceGood + JUMP_MARGIN_IN;
        double turn = Math.abs(Math.atan2(Math.sin(p.heading() - lp.heading()), Math.cos(p.heading() - lp.heading())));
        double maxTurn = 2 * Math.abs(lv.omega) * dtSinceGood + JUMP_MARGIN_RAD;
        return step > maxStep || turn > maxTurn;
    }

    @Override
    public MotionState state() {
        return last != null ? last : inner.state();
    }

    @Override
    public void setPose(Pose pose) {
        inner.setPose(pose);
        last = inner.state();
        lastGoodNs = System.nanoTime();
        consecutive = 0;
    }

    @Override
    public void reset() {
        inner.reset();
    }

    @Override
    public Map<String, Object> debug() {
        return inner.debug();
    }
}
