package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.Map;

/**
 * Wraps the Pinpoint localizer and drops readings that come back as all zeros (pose and velocity
 * exactly 0) while the robot is known to be away from the origin. 2026-11-08: the Line Test logged
 * exact (0,0,0) poses for 1-3 loops mid-path; one made atParametricEnd() fire with 27 in left.
 * A real reading is never exactly 0.0 on all six values while moving, so this can't hide real motion.
 * Pedro's pose()/velocity()/twist() defaults all read state(), so overriding state() covers them.
 */
public class GuardedLocalizer implements Localizer {
    // Only treat all-zero as bad if the last good pose was farther than this from the origin.
    private static final double NEAR_ORIGIN_IN = 0.5;
    // Safety valve: accept zeros after this many in a row (a real Pinpoint reset, not a glitch).
    private static final int MAX_CONSECUTIVE_REJECTS = 10;

    private final Localizer inner;
    private MotionState last;
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
        if (isBad(s) && consecutive < MAX_CONSECUTIVE_REJECTS) {
            consecutive++;
            rejectedReads++;
            return; // keep the last good state
        }
        consecutive = 0;
        last = s;
    }

    private boolean isBad(MotionState s) {
        if (last == null) return false;
        Pose p = s.pose();
        Velocity v = s.velocity();
        boolean allZero = p.x() == 0 && p.y() == 0 && p.heading() == 0
                && v.vx == 0 && v.vy == 0 && v.omega == 0;
        Pose lp = last.pose();
        return allZero && Math.hypot(lp.x(), lp.y()) > NEAR_ORIGIN_IN;
    }

    @Override
    public MotionState state() {
        return last != null ? last : inner.state();
    }

    @Override
    public void setPose(Pose pose) {
        inner.setPose(pose);
        last = inner.state();
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
