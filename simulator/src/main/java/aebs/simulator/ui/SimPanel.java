package aebs.simulator.ui;

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

    // Avoidance behavior:
    // - Ego stays centered unless collision is imminent
    // - When imminent, swerve to the clearer side lane, then return to center
    private static final double LANE_HALF_WIDTH_PX = 70.0;
    private static final double MAX_LATERAL_SPEED_PX_S = 220.0;
    private static final double LATERAL_KP = 6.0;
    private static final double IMMINENT_DIST_M = 18.0;
    private static final double IMMINENT_TTC_S = 1.6;
    private static final double HOLD_AVOID_S = 1.25;

    private double desiredEgoX;
    private double holdAvoidUntilS = 0.0;

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
            updateEgo(dt);
            scenario.step(world, dt);

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

    private void updateEgo(double dt) {
        CarBlock ego = world.ego();
        // Ego should remain fixed longitudinally (centered on screen), only move laterally to avoid collisions.
        double centerX = centerX();

        boolean imminent = imminentCollisionFromSensors();
        if (imminent) {
            desiredEgoX = chooseEscapeX(centerX);
            holdAvoidUntilS = simTimeS + HOLD_AVOID_S;
        } else if (simTimeS >= holdAvoidUntilS) {
            desiredEgoX = centerX;
        }

        // Keep ego within road bounds to avoid "teleporting" outside the lane.
        desiredEgoX = clamp(desiredEgoX, roadLeftPx(), roadRightPx() - ego.aabb().w());

        double err = desiredEgoX - ego.pos().x();
        double vx = clamp(err * LATERAL_KP, -MAX_LATERAL_SPEED_PX_S, MAX_LATERAL_SPEED_PX_S);

        // Keep ego stationary in Y.
        ego.setVel(new Vec2(vx, 0.0));
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

    private boolean imminentCollisionFromSensors() {
        // Use radar readings: distance (m) + relative speed (km/h) via speedKph on the record.
        RadarReading[] radar = sensors.buildRadarReadings(world);

        double bestDistM = Double.POSITIVE_INFINITY;
        double bestClosingMps = 0.0;

        for (RadarReading r : radar) {
            if (r.distanceMetres() < bestDistM) {
                bestDistM = r.distanceMetres();
                // Speed can get noisy; use magnitude so "approaching" doesn't disappear due to sign flips.
                bestClosingMps = Math.abs(r.speedKph()) / 3.6;
            }
        }

        if (bestDistM == Double.POSITIVE_INFINITY) return false;
        if (bestDistM <= IMMINENT_DIST_M) return true;

        if (bestClosingMps <= 1e-3) return false;
        double ttc = bestDistM / bestClosingMps;
        return ttc <= IMMINENT_TTC_S;
    }

    private double chooseEscapeX(double centerX) {
        // Decide whether left or right lane has more clearance ahead.
        double leftX = leftLaneX(centerX);
        double rightX = rightLaneX(centerX);

        double leftClear = laneClearanceAheadPx(leftX);
        double rightClear = laneClearanceAheadPx(rightX);

        // If tie, prefer alternating side based on current position.
        if (leftClear > rightClear) return leftX;
        if (rightClear > leftClear) return rightX;
        return (world.ego().pos().x() <= centerX) ? rightX : leftX;
    }

    private double laneClearanceAheadPx(double laneEgoX) {
        // Estimate min gap to any NPC ahead if ego were at laneEgoX.
        Aabb e = world.ego().aabb();
        double egoW = e.w();
        double egoH = e.h();
        double egoY = e.y();

        // "Virtual" ego AABB at candidate lane X.
        Aabb ve = new Aabb(laneEgoX, egoY, egoW, egoH);

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
        g2.setColor(new Color(24, 27, 34));
        g2.fillRect(w / 2 - 110, 0, 220, h);

        g2.setColor(new Color(44, 50, 62));
        g2.drawLine(w / 2 - 110, 0, w / 2 - 110, h);
        g2.drawLine(w / 2 + 110, 0, w / 2 + 110, h);

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
        g2.drawString(String.format("t=%.2fs  egoSpeed=%.0f px/s  (sensor-based avoidance, SPACE pause)", simTimeS, speed), 10, 18);
        if (collision) {
            g2.setColor(new Color(255, 90, 90));
            g2.drawString("COLLISION", 10, 36);
        }
    }
}

