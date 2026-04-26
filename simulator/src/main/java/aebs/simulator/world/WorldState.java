package aebs.simulator.world;

import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public final class WorldState {
    private final double width;
    private final double height;
    private final CarBlock ego;
    private final List<CarBlock> npcs = new ArrayList<>();
    private final List<PedestrianBlock> pedestrians = new ArrayList<>();

    /** Seconds since simulation start; drives environment and wheel-speed envelope. */
    private double simTimeS = 0.0;

    /** Brake control command (0..1). 0 = no brake, 1 = full brake. */
    private double brakeCommand = 0.0;

    /** Latest wheel-speed feedback (km/h) and its corresponding simulation time. */
    private double lastWheelSpeedKmh = 0.0;
    private double lastWheelSpeedSimTimeS = -1.0;

    /** Escalated driver alert (latched) when braking cannot be verified after corrective attempts. */
    private boolean driverBrakeAlert = false;

    /** Latest fused sensor health status from the perception pipeline. */
    private boolean sensorsHealthy = true;
    private String sensorHealthSummary = "OK";

    /** Fail-safe state when normal operation is degraded. */
    private boolean failSafeActive = false;
    private String failSafeReason = "";

    public WorldState(double width, double height, CarBlock ego) {
        this.width = width;
        this.height = height;
        this.ego = ego;
    }

    public double simTimeS() { return simTimeS; }

    public void setSimTimeS(double simTimeS) {
        this.simTimeS = Math.max(0.0, simTimeS);
    }

    public double brakeCommand() { return brakeCommand; }

    public void setBrakeCommand(double brakeCommand) {
        this.brakeCommand = clamp01(brakeCommand);
    }

    public double lastWheelSpeedKmh() { return lastWheelSpeedKmh; }
    public double lastWheelSpeedSimTimeS() { return lastWheelSpeedSimTimeS; }

    public void setLastWheelSpeedFeedback(double simTimeS, double kmh) {
        this.lastWheelSpeedSimTimeS = simTimeS;
        this.lastWheelSpeedKmh = Math.max(0.0, kmh);
    }

    public boolean driverBrakeAlert() { return driverBrakeAlert; }
    public void setDriverBrakeAlert(boolean v) { this.driverBrakeAlert = v; }

    public boolean sensorsHealthy() { return sensorsHealthy; }
    public String sensorHealthSummary() { return sensorHealthSummary; }

    public void setSensorHealth(boolean ok, String summary) {
        this.sensorsHealthy = ok;
        this.sensorHealthSummary = (summary == null || summary.isBlank()) ? (ok ? "OK" : "FAULT") : summary;
    }

    public boolean failSafeActive() { return failSafeActive; }
    public String failSafeReason() { return failSafeReason; }

    public void setFailSafe(boolean active, String reason) {
        this.failSafeActive = active;
        this.failSafeReason = (reason == null) ? "" : reason;
    }

    public double width() { return width; }
    public double height() { return height; }
    public CarBlock ego() { return ego; }
    public List<CarBlock> npcs() { return Collections.unmodifiableList(npcs); }
    public List<PedestrianBlock> pedestrians() { return Collections.unmodifiableList(pedestrians); }

    public void addNpc(CarBlock npc) { npcs.add(npc); }
    public void removeNpc(CarBlock npc) { npcs.remove(npc); }

    public void addPedestrian(PedestrianBlock p) { pedestrians.add(p); }
    public void removePedestrian(PedestrianBlock p) { pedestrians.remove(p); }

    /** Clears all non-ego actors from the world. */
    public void clearObstacles() {
        npcs.clear();
        pedestrians.clear();
    }

    public List<CarBlock> allCars() {
        List<CarBlock> all = new ArrayList<>(1 + npcs.size());
        all.add(ego);
        all.addAll(npcs);
        return all;
    }

    public Optional<Collidable> firstCollisionWithEgo() {
        Aabb e = ego.aabb();
        for (CarBlock c : npcs) {
            if (e.intersects(c.aabb())) return Optional.of(c);
        }
        for (PedestrianBlock p : pedestrians) {
            if (e.intersects(p.aabb())) return Optional.of(p);
        }
        return Optional.empty();
    }

    public double egoSpeedPixelsPerSec() {
        Vec2 v = ego.vel();
        return Math.hypot(v.x(), v.y());
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}

