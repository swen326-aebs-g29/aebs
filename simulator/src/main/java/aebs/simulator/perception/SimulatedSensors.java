package aebs.simulator.perception;

import aebs.simulator.model.Aabb;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.PedestrianBlock;
import aebs.simulator.world.WorldState;

import java.util.Random;

/**
 * Produces arrays of sensor readings from the animated world.
 * The idea is redundancy: multiple radar returns + multiple camera detections + 4 wheel speeds.
 */
public final class SimulatedSensors {
    private final double pxPerMeter;
    private final double wheelRadiusM;
    private final Random rng;

    public SimulatedSensors(double pxPerMeter, double wheelRadiusM, long seed) {
        this.pxPerMeter = Math.max(1e-6, pxPerMeter);
        this.wheelRadiusM = Math.max(1e-6, wheelRadiusM);
        this.rng = new Random(seed);
    }

    public RadarReading[] buildRadarReadings(WorldState world) {
        long ts = System.currentTimeMillis();

        // For simplicity, return up to 2 radar "tracks" for closest obstacles in-lane.
        CarBlock ego = world.ego();
        Aabb e = ego.aabb();

        double bestDy = Double.POSITIVE_INFINITY;
        double bestRelSpeedPxS = 0.0;
        double secondDy = Double.POSITIVE_INFINITY;
        double secondRelSpeedPxS = 0.0;

        for (CarBlock npc : world.npcs()) {
            Aabb n = npc.aabb();
            boolean xOverlap = (e.x() < n.x() + n.w()) && (e.x() + e.w() > n.x());
            if (!xOverlap) continue;

            double dy = e.y() - (n.y() + n.h());
            if (dy <= 0) continue;

            if (dy < bestDy) {
                secondDy = bestDy;
                secondRelSpeedPxS = bestRelSpeedPxS;
                bestDy = dy;
                bestRelSpeedPxS = npc.vel().y() - ego.vel().y();
            } else if (dy < secondDy) {
                secondDy = dy;
                secondRelSpeedPxS = npc.vel().y() - ego.vel().y();
            }
        }

        for (PedestrianBlock p : world.pedestrians()) {
            Aabb n = p.aabb();
            boolean xOverlap = (e.x() < n.x() + n.w()) && (e.x() + e.w() > n.x());
            if (!xOverlap) continue;

            double dy = e.y() - (n.y() + n.h());
            if (dy <= 0) continue;

            if (dy < bestDy) {
                secondDy = bestDy;
                secondRelSpeedPxS = bestRelSpeedPxS;
                bestDy = dy;
                bestRelSpeedPxS = p.vel().y() - ego.vel().y();
            } else if (dy < secondDy) {
                secondDy = dy;
                secondRelSpeedPxS = p.vel().y() - ego.vel().y();
            }
        }

        RadarReading r1 = bestDy == Double.POSITIVE_INFINITY ? null : radarFromDyAndRelSpeed(bestDy, bestRelSpeedPxS, ts);
        RadarReading r2 = secondDy == Double.POSITIVE_INFINITY ? null : radarFromDyAndRelSpeed(secondDy, secondRelSpeedPxS, ts);

        if (r1 == null && r2 == null) return new RadarReading[0];
        if (r2 == null) return new RadarReading[]{r1};
        if (r1 == null) return new RadarReading[]{r2};
        return new RadarReading[]{r1, r2};
    }

    private RadarReading radarFromDyAndRelSpeed(double dyPx, double relSpeedPxS, long ts) {
        // Convert range (px->m) and relative speed (px/s -> m/s -> kph).
        double rangeM = dyPx / pxPerMeter;
        double relSpeedMps = relSpeedPxS / pxPerMeter;
        double relSpeedKph = relSpeedMps * 3.6;

        // Add mild measurement noise.
        rangeM = Math.max(0.0, rangeM + rng.nextGaussian() * 0.35);
        relSpeedKph = relSpeedKph + rng.nextGaussian() * 0.8;

        return new RadarReading(rangeM, relSpeedKph, ts);
    }

    public CameraReading[] buildCameraReadings(WorldState world) {
        long ts = System.currentTimeMillis();
        CarBlock ego = world.ego();
        Aabb e = ego.aabb();

        int max = 12;
        int total = world.npcs().size() + world.pedestrians().size();

        // Report a classification for each visible object; confidence decreases with distance.
        CameraReading[] tmp = new CameraReading[Math.min(max, total)];
        int k = 0;

        for (CarBlock npc : world.npcs()) {
            if (k >= tmp.length) break;
            Aabb n = npc.aabb();

            double dy = e.y() - (n.y() + n.h());
            if (dy <= 0) continue;
            if (dy > 420) continue;

            if (rng.nextDouble() < 0.06) continue; // miss

            double conf = 1.0 - (dy / 420.0);
            conf = clamp01(conf * (0.70 + rng.nextDouble() * 0.30));
            tmp[k++] = new CameraReading("car", Double.valueOf(conf), Long.valueOf(ts));
        }

        for (PedestrianBlock p : world.pedestrians()) {
            if (k >= tmp.length) break;
            Aabb n = p.aabb();

            double dy = e.y() - (n.y() + n.h());
            if (dy <= 0) continue;
            if (dy > 420) continue;

            if (rng.nextDouble() < 0.06) continue; // miss

            double conf = 1.0 - (dy / 420.0);
            conf = clamp01(conf * (0.65 + rng.nextDouble() * 0.35));
            tmp[k++] = new CameraReading("pedestrian", Double.valueOf(conf), Long.valueOf(ts));
        }

        CameraReading[] out = new CameraReading[k];
        System.arraycopy(tmp, 0, out, 0, k);
        return out;
    }

    public WheelSpeedReading[] buildWheelSpeedReadings(WorldState world) {
        long ts = System.currentTimeMillis();
        double speedPxS = world.egoSpeedPixelsPerSec();
        double speedMps = speedPxS / pxPerMeter;

        // RPM = (linear speed / circumference) * 60
        double circumference = 2.0 * Math.PI * wheelRadiusM;
        double rpm = (speedMps / circumference) * 60.0;

        // Add small per-wheel noise.
        WheelSpeedReading[] wheels = new WheelSpeedReading[4];
        for (int i = 0; i < 4; i++) {
            double noisy = Math.max(0.0, rpm + rng.nextGaussian() * 6.0);
            wheels[i] = new WheelSpeedReading(Double.valueOf(noisy), Long.valueOf(ts));
        }
        return wheels;
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}

