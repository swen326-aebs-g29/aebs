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
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

public final class SimPanel extends JPanel {
    private final WorldState world;
    private final ScenarioEngine scenario;
    private final SimulatedSensors sensors;
    private final Timer timer;

    private long lastNanos = System.nanoTime();
    private boolean paused = false;
    private boolean collision = false;
    private double simTimeS = 0.0;

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
    private static final double CLEARANCE_X_INFLATION_PX = 32.0;
    /** Extra lateral space when deciding if an NPC/ped is “in your path” (wider pass). */
    private static final double PASS_SIDE_MARGIN_PX = 22.0;
    /** Nudge escape targets further toward the left/right inside the road for a side gap. */
    private static final double ESCAPE_SHOULDER_OFFSET_PX = 18.0;
    /** Treat forward gap as smaller so swerve/brake start with more room to the obstacle. */
    private static final double FORWARD_PASS_BUFFER_PX = 45.0;
    private static final double GEO_EARLY_CLEARANCE_PX = EARLY_DIST_M * PX_PER_M;
    private static final double GEO_IMMINENT_CLEARANCE_PX = IMMINENT_DIST_M * PX_PER_M;
    /** Below this gap, scale down forward speed (still swerve when {@code avoid} is true). */
    private static final double BRAKE_CLEARANCE_PX = 260.0;

    private double desiredEgoX;
    private double holdAvoidUntilS = 0.0;
    private final double egoAnchorY;

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
        egoAnchorY = world.ego().pos().y();
    }

    public WorldState world() { return world; }
    public boolean collision() { return collision; }
    public double simTimeS() { return simTimeS; }

    private void tick() {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1_000_000_000.0;
        lastNanos = now;
        dt = Math.max(0.0, Math.min(0.05, dt));

        if (!paused) {
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

        boolean avoid = earlyThreatFromSensors() || imminentCollisionFromSensors()
                || geometricEarlyThreat() || geometricImminentThreat();
        if (avoid) {
            desiredEgoX = chooseEscapeX(centerX);
            holdAvoidUntilS = simTimeS + HOLD_AVOID_S;
        } else if (simTimeS >= holdAvoidUntilS) {
            desiredEgoX = centerX;
        }

        // Keep ego within road bounds to avoid "teleporting" outside the lane.
        desiredEgoX = clamp(desiredEgoX, roadLeftPx(), roadRightPx() - ego.aabb().w());

        double err = desiredEgoX - ego.pos().x();
        double vx = clamp(err * LATERAL_KP, -MAX_LATERAL_SPEED_PX_S, MAX_LATERAL_SPEED_PX_S);

        double clearance = clearanceAheadPx(ego.pos().x());
        double speedFactor = forwardSpeedFactor(clearance);
        ego.setVel(new Vec2(vx, EGO_FORWARD_SPEED_PX_S * speedFactor));
    }

    /** Ground-truth forward gap along current lateral position (cars + pedestrians), no sensor dropout. */
    private double clearanceAheadPx(double egoLeftX) {
        return laneClearanceAheadPx(egoLeftX);
    }

    private boolean geometricEarlyThreat() {
        double c = clearanceAheadPx(world.ego().pos().x());
        return Double.isFinite(c) && c < GEO_EARLY_CLEARANCE_PX;
    }

    private boolean geometricImminentThreat() {
        double c = clearanceAheadPx(world.ego().pos().x());
        return Double.isFinite(c) && c < GEO_IMMINENT_CLEARANCE_PX;
    }

    private static double forwardSpeedFactor(double clearancePx) {
        if (!Double.isFinite(clearancePx)) return 1.0;
        if (clearancePx >= BRAKE_CLEARANCE_PX) return 1.0;
        return clamp(clearancePx / BRAKE_CLEARANCE_PX, 0.12, 1.0);
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
        return threatFromRadar(EARLY_DIST_M, EARLY_TTC_S);
    }

    private boolean imminentCollisionFromSensors() {
        return threatFromRadar(IMMINENT_DIST_M, IMMINENT_TTC_S);
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
        Graphics2D g2 = (Graphics2D) g.create();
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        drawRoad(g2);
        for (CarBlock c : world.npcs()) drawCar(g2, c);
        for (PedestrianBlock p : world.pedestrians()) drawPedestrian(g2, p);
        drawCar(g2, world.ego());
        drawHud(g2);

        g2.dispose();
    }

    private void drawRoad(Graphics2D g2) {
        int w = getWidth();
        int h = getHeight();
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
            g2.fillRect(w / 2 - 2, y + (int) ((-simTimeS * 120) % 36), 4, 18);
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
                "t=%.2fs  egoSpeed=%.0f px/s  wheel=%.0f km/h (0-250)  light=%.2f  camWx=%.2f  radarWx=%.2f  (SPACE pause)",
                simTimeS, speed, wheelKmh, env.ambientLight(), env.cameraWeatherFactor(), env.radarWeatherFactor()), 10, 18);
        int subY = 36;
        if (collision) {
            g2.setColor(new Color(255, 90, 90));
            g2.drawString("COLLISION", 10, subY);
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

