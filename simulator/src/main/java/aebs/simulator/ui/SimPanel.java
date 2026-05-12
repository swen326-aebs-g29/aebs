package aebs.simulator.ui;

import aebs.simulator.environment.DrivingEnvironment;
import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;
import aebs.simulator.perception.RadarReading;
import aebs.simulator.perception.SimulatedSensors;
import aebs.simulator.scenario.ScenarioEngine;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.PedestrianBlock;
import aebs.simulator.world.WorldState;

import javax.swing.JPanel;
import javax.swing.Timer;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Toolkit;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

public final class SimPanel extends JPanel {
    private final WorldState world;
    private final ScenarioEngine scenario;
    private final SimulatedSensors sensors;
    private final Timer timer;

    private final Vec2 initialEgoPos;
    private final Vec2 initialEgoVel;

    private long lastNanos = System.nanoTime();
    private boolean paused = false;
    private boolean frozenOnCollision = false;
    private boolean collision = false;
    private double simTimeS = 0.0;
    private boolean brakingIndicator = false;

    // AEBS controls
    private boolean aebsEnabled = false;
    /** 0..1 — higher = more sensitive (brakes/avoids earlier). */
    private double aebsSensitivity = 0.55;

    // Alert edge detectors (auditory alerts on rising edges).
    private boolean prevEngaged = false;
    private boolean prevHazard = false;
    private boolean prevBraking = false;

    // Maintenance / readiness feedback
    private double sensorFaultSinceS = -1.0;

    // Avoidance behaviour: start lateral move earlier (approach), then hold through imminent window.
    private static final double LANE_HALF_WIDTH_PX = 70.0;
    private static final double MAX_LATERAL_SPEED_PX_S = 140.0;
    private static final double LATERAL_KP = 6.0;
    /** Begin lane change when radar says threat is still farther out */
    private static final double EARLY_DIST_M = 42.0;
    private static final double EARLY_TTC_S = 2.85;
    private static final double IMMINENT_DIST_M = 18.0;
    private static final double IMMINENT_TTC_S = 1.6;
    private static final double HOLD_AVOID_S = 1.25;

    /** px/m in {@link aebs.simulator.SimulatedWorldApp} — geometric thresholds match radar distances without RNG misses. */
    private static final double PX_PER_M = 10.0;
    /** Count obstacles slightly outside the ego lane when estimating forward clearance. */
    private static final double CLEARANCE_X_INFLATION_PX = 40.0;
    /** Extra lateral space when deciding if an NPC/ped is “in your path” (wider pass). */
    private static final double PASS_SIDE_MARGIN_PX = 36.0;
    /** Nudge escape targets further toward the left/right inside the road for a side gap. */
    private static final double ESCAPE_SHOULDER_OFFSET_PX = 28.0;
    /** Treat forward gap as smaller so swerve/brake start with more room to the obstacle. */
    private static final double FORWARD_PASS_BUFFER_PX = 70.0;
    private static final double GEO_EARLY_CLEARANCE_PX = EARLY_DIST_M * PX_PER_M;
    private static final double GEO_IMMINENT_CLEARANCE_PX = IMMINENT_DIST_M * PX_PER_M;
    /** Below this gap, scale down forward speed (still swerve when {@code avoid} is true). */
    private static final double BRAKE_CLEARANCE_PX = 260.0;
    /** Hard minimum lateral separation (pixels) from cars/pedestrians when side-by-side. */
    private static final double HARD_GAP_PX = 22.0;
    /** Only enforce gap against obstacles within this vertical band around ego (pixels). */
    private static final double HARD_GAP_Y_PAD_PX = 70.0;
    /** Cap how fast desiredX can change due to hard-gap (px/s). */
    private static final double HARD_GAP_MAX_SHIFT_PX_S = 140.0;
    /** Additional smoothing on desiredX near obstacles (px/s). */
    private static final double DESIRED_X_MAX_STEP_PX_S = 110.0;

    private double desiredEgoX;
    private double filteredDesiredEgoX;
    private double holdAvoidUntilS = 0.0;
    private final double egoAnchorY;
    private double holdBrakeUntilS = 0.0;

    // Brake supervisor: enforce periodic commands, verification via wheel feedback, corrective attempts, alert.
    private static final double BRAKE_SIGNAL_PERIOD_S = 0.05;     // at least every 50ms
    private static final double BRAKE_VERIFY_WITHIN_S = 0.05;     // verify within 50ms
    private static final double BRAKE_EXEC_MARGIN = 0.05;         // ±5%
    private static final int BRAKE_MAX_CORRECTIVE_ATTEMPTS = 2;

    private double lastBrakeSignalSentS = -1.0;
    private double lastBrakeCmdAtSend = 0.0;
    private double wheelKmhAtSend = 0.0;
    private boolean awaitingBrakeVerification = false;
    private int correctiveBrakeFailures = 0;

    private double brakeOverrideUntilS = 0.0;
    private double brakeOverrideCmd = 0.0;

    // Hazard fallback when sensors are unhealthy: brake and hold a short time.
    private double hazardHoldUntilS = 0.0;
    private double failSafeHoldUntilS = 0.0;

    // Ego forward speed (negative Y is "up" the screen).
    private static final double EGO_FORWARD_SPEED_PX_S = -45.0;

    public SimPanel(WorldState world, ScenarioEngine scenario, SimulatedSensors sensors, int fps) {
        this.world = world;
        this.scenario = scenario;
        this.sensors = sensors;
        setPreferredSize(new Dimension((int) world.width(), (int) world.height()));
        setBackground(new Color(14, 16, 20));
        setFocusable(true);

        addKeyListener(new KeyAdapter() {
            @Override public void keyPressed(KeyEvent e) { handle(e, true); }
            @Override public void keyReleased(KeyEvent e) { handle(e, false); }

            private void handle(KeyEvent e, boolean down) {
                if (e.getKeyCode() == KeyEvent.VK_SPACE && down) paused = !paused;
            }
        });

        int periodMs = Math.max(5, (int) Math.round(1000.0 / Math.max(1, fps)));
        timer = new Timer(periodMs, evt -> tick());
        timer.setCoalesce(true);
        timer.start();

        desiredEgoX = centerX();
        filteredDesiredEgoX = desiredEgoX;
        egoAnchorY = world.ego().pos().y();

        initialEgoPos = world.ego().pos();
        initialEgoVel = world.ego().vel();
    }

    public WorldState world() { return world; }
    public boolean collision() { return collision; }
    public double simTimeS() { return simTimeS; }

    public boolean aebsEnabled() { return aebsEnabled; }
    public void setAebsEnabled(boolean v) { this.aebsEnabled = v; }

    public double aebsSensitivity() { return aebsSensitivity; }
    public void setAebsSensitivity(double v) { this.aebsSensitivity = clamp(v, 0.0, 1.0); }

    public void restart() {
        // Reset UI / time
        paused = false;
        frozenOnCollision = false;
        collision = false;
        simTimeS = 0.0;
        lastNanos = System.nanoTime();

        // Reset world state
        world.clearObstacles();
        world.setSimTimeS(0.0);
        world.setBrakeCommand(0.0);
        world.setDriverBrakeAlert(false);
        world.setSensorHealth(true, "OK");
        world.setFailSafe(false, "");

        // Reset ego pose/vel
        CarBlock ego = world.ego();
        Vec2 cur = ego.pos();
        ego.translate(initialEgoPos.x() - cur.x(), initialEgoPos.y() - cur.y());
        ego.setVel(initialEgoVel);
        ego.setCollided(false);

        // Reset planner / holds
        desiredEgoX = centerX();
        filteredDesiredEgoX = desiredEgoX;
        holdAvoidUntilS = 0.0;
        holdBrakeUntilS = 0.0;
        hazardHoldUntilS = 0.0;
        failSafeHoldUntilS = 0.0;
        sensorFaultSinceS = -1.0;

        // Reset alerts and brake supervisor state
        prevEngaged = false;
        prevHazard = false;
        prevBraking = false;
        brakingIndicator = false;
        lastBrakeSignalSentS = -1.0;
        lastBrakeCmdAtSend = 0.0;
        wheelKmhAtSend = 0.0;
        awaitingBrakeVerification = false;
        correctiveBrakeFailures = 0;
        brakeOverrideUntilS = 0.0;
        brakeOverrideCmd = 0.0;

        // Reset scenario + sensors internal state
        scenario.reset();
        sensors.reset();

        repaint();
        requestFocusInWindow();
    }

    private void tick() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1_000_000_000.0;
        lastNanos = now;
        dt = Math.max(0.0, Math.min(0.05, dt));

        if (!paused && !frozenOnCollision) {
            simTimeS += dt;
            world.setSimTimeS(simTimeS);
            updateEgo(dt);
            scenario.step(world, dt);

            // Camera lock: keep ego at a fixed screen Y and scroll the world instead.
            lockEgoYAndScrollWorld();

            // Remove NPCs that went past bottom of screen
            for (CarBlock npc : world.npcs().toArray(new CarBlock[0])) {
                if (npc.pos().y() > world.height() + 100) world.removeNpc(npc);
            }
            for (PedestrianBlock p : world.pedestrians().toArray(new PedestrianBlock[0])) {
                if (p.pos().y() > world.height() + 100) world.removePedestrian(p);
            }

            collision = world.firstCollisionWithEgo().isPresent();
            if (collision) {
                world.ego().setCollided(true);
                world.firstCollisionWithEgo().ifPresent(n -> n.setCollided(true));
                world.setBrakeCommand(1.0);
                world.ego().setVel(new Vec2(0.0, 0.0));
                paused = true;
                frozenOnCollision = true;
            } else {
                world.ego().setCollided(false);
                for (CarBlock npc : world.npcs()) npc.setCollided(false);
                for (PedestrianBlock p : world.pedestrians()) p.setCollided(false);
            }
        }

        repaint();
    }

    private void lockEgoYAndScrollWorld() {
        CarBlock ego = world.ego();
        double dy = egoAnchorY - ego.pos().y();
        if (Math.abs(dy) < 1e-6) return;

        // Shift everything by dy so ego returns to its anchor Y.
        ego.translate(0.0, dy);
        for (CarBlock npc : world.npcs()) npc.translate(0.0, dy);
        for (PedestrianBlock p : world.pedestrians()) p.translate(0.0, dy);
    }

    private void updateEgo(double dt) {
        CarBlock ego = world.ego();
        // Ego should remain fixed longitudinally (centered on screen), only move laterally to avoid collisions.
        double centerX = centerX();

        boolean avoid = aebsEnabled && (earlyThreatFromSensors() || imminentCollisionFromSensors()
                || geometricEarlyThreat() || geometricImminentThreat());
        if (avoid) {
            desiredEgoX = chooseEscapeX(centerX);
            holdAvoidUntilS = simTimeS + HOLD_AVOID_S;
        } else if (simTimeS >= holdAvoidUntilS) {
            desiredEgoX = centerX;
        }

        // Keep ego within road bounds to avoid "teleporting" outside the lane.
        double rawDesiredX = clamp(desiredEgoX, roadLeftPx(), roadRightPx() - ego.aabb().w());
        rawDesiredX = enforceHardMinimumLateralGap(rawDesiredX, dt);

        // Smooth target changes to prevent chatter near constraints.
        filteredDesiredEgoX = moveToward(filteredDesiredEgoX, rawDesiredX, DESIRED_X_MAX_STEP_PX_S * dt);
        filteredDesiredEgoX = clamp(filteredDesiredEgoX, roadLeftPx(), roadRightPx() - ego.aabb().w());
        // Ensure safety after smoothing (projection is stable; no extra rate limit here).
        filteredDesiredEgoX = enforceHardMinimumLateralGap(filteredDesiredEgoX, dt);
        desiredEgoX = filteredDesiredEgoX;

        double err = desiredEgoX - ego.pos().x();
        double vx = clamp(err * LATERAL_KP, -MAX_LATERAL_SPEED_PX_S, MAX_LATERAL_SPEED_PX_S);

        double clearance = clearanceAheadPx(ego.pos().x());
        double speedFactor = forwardSpeedFactor(clearance);

        // If the system believes collision is imminent, command an immediate stop for a short window.
        // This is the last line of defense before impact.
        if (imminentCollisionFromSensors() || geometricImminentThreat()) {
            holdBrakeUntilS = Math.max(holdBrakeUntilS, simTimeS + 0.35);
        }

        boolean emergencyHold = simTimeS < holdBrakeUntilS;
        double plannedBrakeCmd = emergencyHold ? 1.0 : (1.0 - speedFactor);

        // Fail-safe mechanisms: if normal operation is degraded, prefer a safe brake hold and center.
        boolean sensorFault = !world.sensorsHealthy();
        boolean brakeFault = world.driverBrakeAlert();
        if (sensorFault) {
            hazardHoldUntilS = Math.max(hazardHoldUntilS, simTimeS + 0.6);
        }
        if (brakeFault) {
            failSafeHoldUntilS = Math.max(failSafeHoldUntilS, simTimeS + 1.2);
        }

        boolean hazardHold = simTimeS < hazardHoldUntilS;
        boolean failSafeHold = simTimeS < failSafeHoldUntilS;
        boolean degraded = hazardHold || failSafeHold;
        world.setFailSafe(degraded, failSafeHold ? "brake_unverified" : (hazardHold ? "sensor_fault" : ""));

        if (degraded) {
            plannedBrakeCmd = 1.0;
            // When degraded, stop trying to swerve aggressively; stay near center.
            desiredEgoX = moveToward(desiredEgoX, centerX, 60.0 * dt);
        }

        // Apply any corrective override requested by brake supervisor.
        if (simTimeS >= brakeOverrideUntilS) {
            brakeOverrideCmd = 0.0;
        }
        double effectiveBrakeCmd = Math.max(plannedBrakeCmd, brakeOverrideCmd);
        effectiveBrakeCmd = clamp(effectiveBrakeCmd, 0.0, 1.0);
        double effectiveSpeedFactor = clamp(1.0 - effectiveBrakeCmd, 0.0, 1.0);

        world.setBrakeCommand(effectiveBrakeCmd);
        superviseBraking(effectiveBrakeCmd);

        // Reverted: longitudinal speed snaps to target factor (brakeCmd still published).
        double vy = emergencyHold ? 0.0 : (EGO_FORWARD_SPEED_PX_S * effectiveSpeedFactor);
        ego.setVel(new Vec2(vx, vy));

        // HUD: only flag "BRAKING: YES" when the ego is both slowing down AND moving laterally
        // to get out of the way of obstacles.
        boolean movingOutOfWay = Math.abs(vx) > 10.0;
        boolean slowingNow = emergencyHold || effectiveBrakeCmd >= 0.30 || effectiveSpeedFactor < 0.95;
        brakingIndicator = movingOutOfWay && slowingNow && !collision && !frozenOnCollision;

        // Alerts (auditory + visual state)
        boolean brakingNow = effectiveBrakeCmd > 0.05 || emergencyHold;
        boolean hazardNow = collision || degraded || (aebsEnabled && (imminentCollisionFromSensors() || geometricImminentThreat()));
        boolean engagedNow = aebsEnabled && (avoid || brakingNow || degraded);

        if (!prevEngaged && engagedNow) Toolkit.getDefaultToolkit().beep();
        if (!prevHazard && hazardNow) { Toolkit.getDefaultToolkit().beep(); Toolkit.getDefaultToolkit().beep(); }
        if (!prevBraking && brakingNow) Toolkit.getDefaultToolkit().beep();
        prevEngaged = engagedNow;
        prevHazard = hazardNow;
        prevBraking = brakingNow;

        if (!world.sensorsHealthy()) {
            if (sensorFaultSinceS < 0.0) sensorFaultSinceS = simTimeS;
        } else {
            sensorFaultSinceS = -1.0;
        }
    }

    /**
     * Ensures the ego keeps a minimum side gap to nearby objects (cars + pedestrians) when
     * they are roughly alongside in Y. This prevents "door-to-door" passes even if the planner
     * chose a lane that technically avoids intersection.
     */
    private double enforceHardMinimumLateralGap(double candidateLeftX) {
        Aabb egoBox = world.ego().aabb();
        double egoW = egoBox.w();
        double yMin = egoBox.y() - HARD_GAP_Y_PAD_PX;
        double yMax = egoBox.y() + egoBox.h() + HARD_GAP_Y_PAD_PX;

        double roadLeft = roadLeftPx();
        double roadRight = roadRightPx() - egoW;

        double x = clamp(candidateLeftX, roadLeft, roadRight);

        // Build forbidden x-intervals for egoLeftX such that ego would violate the side gap.
        // Ego is allowed when (egoRight <= oLeft-gap) OR (egoLeft >= oRight+gap).
        // Therefore forbidden is (oLeft-gap-egoW, oRight+gap).
        Interval[] tmp = new Interval[world.npcs().size() + world.pedestrians().size()];
        int k = 0;

        for (CarBlock npc : world.npcs()) {
            Aabb o = npc.aabb();
            if ((o.y() > yMax) || (o.y() + o.h() < yMin)) continue;
            double lo = (o.x() - HARD_GAP_PX - egoW);
            double hi = (o.x() + o.w() + HARD_GAP_PX);
            tmp[k++] = new Interval(lo, hi);
        }
        for (PedestrianBlock p : world.pedestrians()) {
            Aabb o = p.aabb();
            if ((o.y() > yMax) || (o.y() + o.h() < yMin)) continue;
            double lo = (o.x() - HARD_GAP_PX - egoW);
            double hi = (o.x() + o.w() + HARD_GAP_PX);
            tmp[k++] = new Interval(lo, hi);
        }

        if (k == 0) return x;

        // Clamp forbidden intervals to the drivable domain.
        for (int i = 0; i < k; i++) {
            Interval in = tmp[i];
            double lo = clamp(in.lo, roadLeft, roadRight);
            double hi = clamp(in.hi, roadLeft, roadRight);
            tmp[i] = new Interval(Math.min(lo, hi), Math.max(lo, hi));
        }

        Interval[] merged = mergeIntervals(tmp, k);

        // If x is inside any forbidden interval, project it to the nearest boundary.
        double projected = projectOutOfForbidden(x, merged);
        if (Math.abs(projected - x) < 1e-6) return x;

        // Rate limit the correction to avoid oscillation.
        double maxShift = HARD_GAP_MAX_SHIFT_PX_S * Math.max(0.0, dtSeconds);
        double dx = clamp(projected - x, -maxShift, maxShift);
        return clamp(x + dx, roadLeft, roadRight);
    }

    private double enforceHardMinimumLateralGap(double candidateLeftX, double dtSeconds) {
        // wrapper so we can rate-limit based on dt
        this.dtSeconds = dtSeconds;
        return enforceHardMinimumLateralGap(candidateLeftX);
    }

    // Stored per-call to allow dt-based cap without threading dt through all helpers.
    private double dtSeconds = 0.016;

    private static double moveToward(double cur, double target, double maxStep) {
        if (maxStep <= 0) return cur;
        double d = target - cur;
        if (Math.abs(d) <= maxStep) return target;
        return cur + Math.signum(d) * maxStep;
    }

    private record Interval(double lo, double hi) {}

    private static Interval[] mergeIntervals(Interval[] in, int n) {
        // Simple insertion sort by lo (n is tiny in this sim).
        for (int i = 1; i < n; i++) {
            Interval key = in[i];
            int j = i - 1;
            while (j >= 0 && in[j].lo > key.lo) {
                in[j + 1] = in[j];
                j--;
            }
            in[j + 1] = key;
        }

        Interval[] out = new Interval[n];
        int k = 0;
        Interval cur = in[0];
        for (int i = 1; i < n; i++) {
            Interval nx = in[i];
            if (nx.lo <= cur.hi) {
                cur = new Interval(cur.lo, Math.max(cur.hi, nx.hi));
            } else {
                out[k++] = cur;
                cur = nx;
            }
        }
        out[k++] = cur;

        Interval[] trimmed = new Interval[k];
        System.arraycopy(out, 0, trimmed, 0, k);
        return trimmed;
    }

    private static double projectOutOfForbidden(double x, Interval[] forbidden) {
        for (Interval f : forbidden) {
            if (x >= f.lo && x <= f.hi) {
                double toLo = Math.abs(x - f.lo);
                double toHi = Math.abs(f.hi - x);
                return (toLo <= toHi) ? f.lo : f.hi;
            }
        }
        return x;
    }

    private void superviseBraking(double brakeCmd) {
        boolean activeBraking = brakeCmd > 0.02;
        if (!activeBraking) {
            awaitingBrakeVerification = false;
            correctiveBrakeFailures = 0;
            world.setDriverBrakeAlert(false);
            return;
        }

        // Send a brake control "signal" at least every 50ms (we model this as updating the command).
        if (lastBrakeSignalSentS < 0.0 || (simTimeS - lastBrakeSignalSentS) >= BRAKE_SIGNAL_PERIOD_S) {
            lastBrakeSignalSentS = simTimeS;
            lastBrakeCmdAtSend = brakeCmd;
            wheelKmhAtSend = world.lastWheelSpeedKmh();
            awaitingBrakeVerification = true;
        }

        // Verify brake activation via sensor feedback within 50ms of sending the command.
        if (awaitingBrakeVerification && (simTimeS - lastBrakeSignalSentS) >= BRAKE_VERIFY_WITHIN_S) {
            // Require feedback sample that is at/after the command time (or very close).
            if (world.lastWheelSpeedSimTimeS() < (lastBrakeSignalSentS - 1e-6)) {
                // No feedback yet; keep waiting (next tick still satisfies "within 50ms" in this sim loop).
                return;
            }

            double dt = BRAKE_VERIFY_WITHIN_S;
            double expectedDrop = SimulatedSensors.WHEEL_EXPECTED_DECEL_FULL_BRAKE_KMH_S * lastBrakeCmdAtSend * dt;
            expectedDrop = Math.max(0.0, expectedDrop);
            double actualDrop = wheelKmhAtSend - world.lastWheelSpeedKmh();

            boolean ok;
            if (expectedDrop < 1e-6) {
                ok = true;
            } else {
                double ratio = actualDrop / expectedDrop;
                ok = ratio >= (1.0 - BRAKE_EXEC_MARGIN) && ratio <= (1.0 + BRAKE_EXEC_MARGIN);
            }

            awaitingBrakeVerification = false;
            if (ok) {
                correctiveBrakeFailures = 0;
                world.setDriverBrakeAlert(false);
            } else {
                correctiveBrakeFailures++;
                if (correctiveBrakeFailures <= BRAKE_MAX_CORRECTIVE_ATTEMPTS) {
                    // Corrective attempt: command stronger braking for a short window.
                    brakeOverrideCmd = 1.0;
                    brakeOverrideUntilS = simTimeS + 0.25;
                } else {
                    // Escalate an alert to the driver.
                    world.setDriverBrakeAlert(true);
                }
            }
        }
    }

    /** Ground-truth forward gap along current lateral position (cars + pedestrians), no sensor dropout. */
    private double clearanceAheadPx(double egoLeftX) {
        return laneClearanceAheadPx(egoLeftX);
    }

    private boolean geometricEarlyThreat() {
        double c = clearanceAheadPx(world.ego().pos().x());
        return Double.isFinite(c) && c < (earlyDistM() * PX_PER_M);
    }

    private boolean geometricImminentThreat() {
        double c = clearanceAheadPx(world.ego().pos().x());
        return Double.isFinite(c) && c < (imminentDistM() * PX_PER_M);
    }

    private double forwardSpeedFactor(double clearancePx) {
        if (!Double.isFinite(clearancePx)) return 1.0;
        double brakeClear = brakeClearancePx();
        if (clearancePx >= brakeClear) return 1.0;
        return clamp(clearancePx / brakeClear, 0.12, 1.0);
    }

    private double earlyDistM() {
        // More sensitivity => treat hazards as "closer" sooner (react earlier).
        // Keep mid-range (50%) clearly more conservative than baseline.
        return EARLY_DIST_M * (1.0 + 0.9 * aebsSensitivity);
    }

    private double imminentDistM() {
        return IMMINENT_DIST_M * (1.0 + 0.7 * aebsSensitivity);
    }

    private double brakeClearancePx() {
        return BRAKE_CLEARANCE_PX * (1.1 + 1.0 * aebsSensitivity);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double roadLeftPx() {
        return world.width() / 2.0 - 110.0;
    }

    private double roadRightPx() {
        return world.width() / 2.0 + 110.0;
    }

    private double centerX() {
        Aabb e = world.ego().aabb();
        double egoW = e.w();
        return (world.width() / 2.0) - (egoW / 2.0);
    }

    private double leftLaneX(double centerX) {
        return centerX - LANE_HALF_WIDTH_PX;
    }

    private double rightLaneX(double centerX) {
        return centerX + LANE_HALF_WIDTH_PX;
    }

    private boolean earlyThreatFromSensors() {
        return aebsEnabled && threatFromRadar(earlyDistM(), EARLY_TTC_S);
    }

    private boolean imminentCollisionFromSensors() {
        return aebsEnabled && threatFromRadar(imminentDistM(), IMMINENT_TTC_S);
    }

    private boolean threatFromRadar(double distThresholdM, double ttcThresholdS) {
        RadarReading[] radar = sensors.buildRadarReadings(world);

        double bestDistM = Double.POSITIVE_INFINITY;
        double bestClosingMps = 0.0;

        for (RadarReading r : radar) {
            if (r.distanceMetres() < bestDistM) {
                bestDistM = r.distanceMetres();
                bestClosingMps = Math.abs(r.speedKph()) / 3.6;
            }
        }

        if (bestDistM == Double.POSITIVE_INFINITY) return false;
        if (bestDistM <= distThresholdM) return true;

        if (bestClosingMps <= 1e-3) return false;
        double ttc = bestDistM / bestClosingMps;
        return ttc <= ttcThresholdS;
    }

    private double chooseEscapeX(double centerX) {
        // Decide whether left or right lane has more clearance ahead.
        double leftX = leftLaneX(centerX) - ESCAPE_SHOULDER_OFFSET_PX;
        double rightX = rightLaneX(centerX) + ESCAPE_SHOULDER_OFFSET_PX;

        double leftClear = laneClearanceAheadPx(leftX);
        double rightClear = laneClearanceAheadPx(rightX);

        // If tie, prefer alternating side based on current position.
        if (leftClear > rightClear) return clamp(leftX, roadLeftPx(), roadRightPx() - world.ego().aabb().w());
        if (rightClear > leftClear) return clamp(rightX, roadLeftPx(), roadRightPx() - world.ego().aabb().w());
        double tie = (world.ego().pos().x() <= centerX) ? rightX : leftX;
        return clamp(tie, roadLeftPx(), roadRightPx() - world.ego().aabb().w());
    }

    private double laneClearanceAheadPx(double laneEgoX) {
        // Estimate min gap to any NPC ahead if ego were at laneEgoX.
        Aabb e = world.ego().aabb();
        double lateralPad = CLEARANCE_X_INFLATION_PX + PASS_SIDE_MARGIN_PX;
        double egoW = e.w() + 2.0 * lateralPad;
        double egoH = e.h();
        double egoY = e.y();

        // "Virtual" ego AABB at candidate lane X (inflated laterally for a passing gap).
        Aabb ve = new Aabb(laneEgoX - lateralPad, egoY, egoW, egoH);

        double bestDy = Double.POSITIVE_INFINITY;
        for (CarBlock npc : world.npcs()) {
            Aabb n = npc.aabb();
            boolean xOverlap = (ve.x() < n.x() + n.w()) && (ve.x() + ve.w() > n.x());
            if (!xOverlap) continue;

            double dy = ve.y() - (n.y() + n.h());
            if (dy <= 0) continue;
            if (dy < bestDy) bestDy = dy;
        }

        for (PedestrianBlock p : world.pedestrians()) {
            Aabb n = p.aabb();
            boolean xOverlap = (ve.x() < n.x() + n.w()) && (ve.x() + ve.w() > n.x());
            if (!xOverlap) continue;

            double dy = ve.y() - (n.y() + n.h());
            if (dy <= 0) continue;
            if (dy < bestDy) bestDy = dy;
        }
        if (bestDy < Double.POSITIVE_INFINITY) {
            bestDy = Math.max(0.0, bestDy - FORWARD_PASS_BUFFER_PX);
        }
        return bestDy;
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        // Draw the world in a fixed coordinate system (world.width/height) but scale it to the
        // current panel size so resizing the window "fills" the screen.
        Graphics2D worldG = (Graphics2D) g.create();
        worldG.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        double sx = getWidth() / Math.max(1.0, world.width());
        double sy = getHeight() / Math.max(1.0, world.height());
        worldG.scale(sx, sy);

        drawRoad(worldG);
        for (CarBlock c : world.npcs()) drawCar(worldG, c);
        for (PedestrianBlock p : world.pedestrians()) drawPedestrian(worldG, p);
        drawCar(worldG, world.ego());
        worldG.dispose();

        // HUD stays in screen pixels (unscaled) for readability.
        Graphics2D hudG = (Graphics2D) g.create();
        hudG.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        drawHud(hudG);
        hudG.dispose();
    }

    private void drawRoad(Graphics2D g2) {
        int w = (int) Math.round(world.width());
        int h = (int) Math.round(world.height());
        int roadX = w / 2 - 110;
        int roadW = 220;

        // Main road surface
        g2.setColor(new Color(24, 27, 34));
        g2.fillRect(roadX, 0, roadW, h);

        g2.setColor(new Color(44, 50, 62));
        g2.drawLine(roadX, 0, roadX, h);
        g2.drawLine(roadX + roadW, 0, roadX + roadW, h);

        // dashed lane marker
        g2.setColor(new Color(180, 180, 180, 110));
        for (int y = 0; y < h; y += 36) {
            g2.fillRect(w / 2 - 2, y + (int) ((simTimeS * 120) % 36), 4, 18);
        }
    }

    private void drawCar(Graphics2D g2, CarBlock c) {
        Aabb a = c.aabb();
        boolean ego = c.kind() == CarBlock.Kind.EGO;

        Color base = ego ? new Color(64, 210, 255) : new Color(255, 176, 84);
        if (c.collided()) base = new Color(255, 70, 70);

        g2.setColor(base.darker());
        g2.fillRoundRect((int) a.x(), (int) a.y(), (int) a.w(), (int) a.h(), 10, 10);

        g2.setColor(base);
        g2.drawRoundRect((int) a.x(), (int) a.y(), (int) a.w(), (int) a.h(), 10, 10);

        // windshield stripe
        g2.setColor(new Color(255, 255, 255, 50));
        g2.fillRoundRect((int) a.x() + 6, (int) a.y() + 8, (int) a.w() - 12, 10, 8, 8);
    }

    private void drawPedestrian(Graphics2D g2, PedestrianBlock p) {
        // Stickman: head + torso + arms/legs.
        Aabb a = p.aabb();

        int xMid = (int) (a.x() + a.w() / 2.0);
        int yTop = (int) a.y();

        boolean collided = p.collided();
        Color base = collided ? new Color(255, 70, 70) : new Color(240, 240, 240);
        g2.setColor(base);

        // head
        int headR = Math.max(4, (int) (a.w() * 0.45));
        g2.fillOval(xMid - headR, yTop, 2 * headR, 2 * headR);

        int neckY = yTop + 2 * headR;
        int torsoBottomY = (int) (a.y() + a.h() * 0.62);
        int torsoBottomX = xMid;

        // body
        g2.drawLine(torsoBottomX, neckY, torsoBottomX, torsoBottomY);

        // arms
        int armY = neckY + (torsoBottomY - neckY) / 3;
        int armSpread = (int) (a.w() * 0.9);
        g2.drawLine(torsoBottomX, armY, torsoBottomX - armSpread / 2, armY + 6);
        g2.drawLine(torsoBottomX, armY, torsoBottomX + armSpread / 2, armY + 6);

        // legs
        int legSpreadY = (int) (a.h() * 0.20);
        g2.drawLine(torsoBottomX, torsoBottomY, torsoBottomX - 4, torsoBottomY + legSpreadY);
        g2.drawLine(torsoBottomX, torsoBottomY, torsoBottomX + 4, torsoBottomY + legSpreadY);
    }

    private void drawHud(Graphics2D g2) {
        g2.setFont(new Font("Menlo", Font.PLAIN, 12));
        g2.setColor(new Color(220, 228, 240));
        double speed = world.egoSpeedPixelsPerSec();
        double wheelKmh = sensors.simulatedWheelSpeedKmh(world);
        DrivingEnvironment env = DrivingEnvironment.forSimTime(simTimeS);
        g2.drawString(String.format(
                "t=%.2fs  egoSpeed=%.0f px/s  wheel=%.0f km/h (0-250)  AEBS=%s  sens=%.0f%%  READY=%s  light=%.2f  camWx=%.2f  radarWx=%.2f  (SPACE pause)",
                simTimeS, speed, wheelKmh,
                (aebsEnabled ? "ON" : "OFF"),
                aebsSensitivity * 100.0,
                (aebsEnabled && world.sensorsHealthy() && !world.failSafeActive() ? "YES" : "NO"),
                env.ambientLight(), env.cameraWeatherFactor(), env.radarWeatherFactor()), 10, 18);
        int subY = 36;
        if (collision) {
            g2.setColor(new Color(255, 90, 90));
            g2.drawString("COLLISION", 10, subY);
            subY += 18;
        }
        if (world.driverBrakeAlert()) {
            g2.setColor(new Color(255, 120, 120));
            g2.drawString("ALERT: braking not verified (wheel feedback)", 10, subY);
            subY += 18;
        }
        if (!world.sensorsHealthy()) {
            g2.setColor(new Color(255, 170, 90));
            g2.drawString("SENSOR FAULT: " + world.sensorHealthSummary(), 10, subY);
            subY += 18;
        }
        if (world.failSafeActive()) {
            g2.setColor(new Color(255, 210, 120));
            g2.drawString("FAIL-SAFE ACTIVE: " + world.failSafeReason(), 10, subY);
            subY += 18;
        }
        // Visual feedback for immediate braking actions.
        g2.setColor(brakingIndicator ? new Color(190, 225, 255) : new Color(170, 178, 190));
        g2.drawString(brakingIndicator ? "BRAKING: YES" : "BRAKING: NO", 10, subY);
        subY += 18;
        if (sensorFaultSinceS >= 0.0 && (simTimeS - sensorFaultSinceS) > 2.0) {
            g2.setColor(new Color(255, 150, 90));
            g2.drawString("MAINTENANCE: check sensors / cleaning", 10, subY);
            subY += 18;
        }
        if (sensors.severeDecelerationTractionConcern()) {
            g2.setColor(new Color(255, 200, 120));
            g2.drawString(String.format("WHEEL: severe decel / traction split  (%.0f km/h/s)",
                    sensors.longitudinalDecelerationKmhPerS()), 10, subY);
        } else if (sensors.rapidDecelerationBrakingConcern()) {
            g2.setColor(new Color(200, 210, 240));
            g2.drawString(String.format("WHEEL: rapid decel braking  (%.0f km/h/s)",
                    sensors.longitudinalDecelerationKmhPerS()), 10, subY);
        }
    }
}

