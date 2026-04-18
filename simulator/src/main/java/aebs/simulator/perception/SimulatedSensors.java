package aebs.simulator.perception;

import aebs.simulator.environment.DrivingEnvironment;
import aebs.simulator.model.Aabb;
import aebs.simulator.model.Vec2;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.PedestrianBlock;
import aebs.simulator.world.WorldState;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

/**
 * Produces arrays of sensor readings from the animated world.
 * The idea is redundancy: multiple radar returns + multiple camera detections + 4 wheel speeds.
 */
public final class SimulatedSensors {
    /**
     * Declared radar/lidar measurement envelope (metres): reported readings are simulated to fall
     * in this band. Geometry is not clipped to these distances — true range is computed from the
     * world, then noise is applied and the value is mapped into this envelope for output.
     */
    public static final double RADAR_MIN_RANGE_M = 0.5;
    public static final double RADAR_MAX_RANGE_M = 200.0;

    /** Simulated wheel-speed envelope (km/h): synthesized vehicle speed before RPM conversion. */
    public static final double WHEEL_SPEED_MIN_KMH = 0.0;
    public static final double WHEEL_SPEED_MAX_KMH = 250.0;

    /**
     * Operational thresholds on longitudinal deceleration (ego speed loss, km/h per second).
     * Above rapid: wider RPM spread across wheels (heavy braking); above severe: stronger split (slip).
     */
    public static final double WHEEL_RAPID_DECEL_THRESHOLD_KMH_S = 55.0;
    public static final double WHEEL_SEVERE_DECEL_THRESHOLD_KMH_S = 95.0;

    private final double pxPerMeter;
    private final double wheelRadiusM;
    private final Random rng;

    private double prevEgoSpeedKmhForDecel = -1.0;
    private double prevDecelSimTimeS = -1.0;
    private double lastLongitudinalDecelKmhPerS = 0.0;

    public SimulatedSensors(double pxPerMeter, double wheelRadiusM, long seed) {
        this.pxPerMeter = Math.max(1e-6, pxPerMeter);
        this.wheelRadiusM = Math.max(1e-6, wheelRadiusM);
        this.rng = new Random(seed);
    }

    /** Latest estimated ego longitudinal deceleration magnitude (km/h per second), non-negative. */
    public double longitudinalDecelerationKmhPerS() {
        return lastLongitudinalDecelKmhPerS;
    }

    public boolean rapidDecelerationBrakingConcern() {
        return lastLongitudinalDecelKmhPerS >= WHEEL_RAPID_DECEL_THRESHOLD_KMH_S;
    }

    public boolean severeDecelerationTractionConcern() {
        return lastLongitudinalDecelKmhPerS >= WHEEL_SEVERE_DECEL_THRESHOLD_KMH_S;
    }

    public RadarReading[] buildRadarReadings(WorldState world) {
        long ts = System.currentTimeMillis();
        DrivingEnvironment env = DrivingEnvironment.forSimTime(world.simTimeS());

        CarBlock ego = world.ego();
        Aabb e = ego.aabb();

        List<RadarTrack> tracks = new ArrayList<>();

        for (CarBlock npc : world.npcs()) {
            maybeAddRadarTrack(e, npc.aabb(), npc.vel().y() - ego.vel().y(), env, ts, tracks);
        }
        for (PedestrianBlock p : world.pedestrians()) {
            maybeAddRadarTrack(e, p.aabb(), p.vel().y() - ego.vel().y(), env, ts, tracks);
        }

        tracks.sort(Comparator.comparingDouble(RadarTrack::rangeM));
        if (tracks.isEmpty()) return new RadarReading[0];
        if (tracks.size() == 1) return new RadarReading[]{tracks.get(0).toReading()};
        return new RadarReading[]{tracks.get(0).toReading(), tracks.get(1).toReading()};
    }

    private void maybeAddRadarTrack(
            Aabb egoBox,
            Aabb target,
            double relSpeedPxS,
            DrivingEnvironment env,
            long ts,
            List<RadarTrack> out
    ) {
        // "In front" corridor check: allow sensing even when not perfectly overlapping in X
        // so ego can react earlier to upcoming obstacles.
        double egoCx = egoBox.centerX();
        double targetCx = target.centerX();
        double corridorHalfWidthPx = (egoBox.w() * 0.5) + 85.0;
        if (Math.abs(targetCx - egoCx) > corridorHalfWidthPx) return;

        double dyPx = egoBox.y() - (target.y() + target.h());
        if (dyPx <= 0) return;

        // True geometric range (simulation); not limited to the operational envelope.
        double geomRangeM = dyPx / pxPerMeter;

        double wM = target.w() / pxPerMeter;
        double hM = target.h() / pxPerMeter;
        double sizeM2 = Math.max(1e-6, wM * hM);

        // Detection likelihood uses the same curve as the physical sensor spec window, without
        // discarding geometry outside 0.5–200 m (beyond max: attenuate slightly).
        double rangeForProb = clamp(geomRangeM, RADAR_MIN_RANGE_M, RADAR_MAX_RANGE_M);
        double pDet = radarDetectionProbability(rangeForProb, sizeM2, env);
        if (geomRangeM > RADAR_MAX_RANGE_M) {
            double over = geomRangeM - RADAR_MAX_RANGE_M;
            pDet *= clamp(1.0 - over / (RADAR_MAX_RANGE_M * 1.5), 0.06, 1.0);
        }
        if (geomRangeM < RADAR_MIN_RANGE_M) {
            pDet *= 0.82;
        }
        if (rng.nextDouble() > pDet) {
            return;
        }

        double relSpeedMps = relSpeedPxS / pxPerMeter;
        double relSpeedKph = relSpeedMps * 3.6;

        double rangeSigma = 0.28 + (1.0 - env.radarWeatherFactor()) * 1.15;
        double speedSigma = 0.65 + (1.0 - env.radarWeatherFactor()) * 1.4;

        // Simulated reported range: noisy, then folded into the published 0.5–200 m envelope.
        double reportedM = geomRangeM + rng.nextGaussian() * rangeSigma;
        relSpeedKph = relSpeedKph + rng.nextGaussian() * speedSigma;

        reportedM = clamp(reportedM, RADAR_MIN_RANGE_M, RADAR_MAX_RANGE_M);
        out.add(new RadarTrack(reportedM, relSpeedKph, ts));
    }

    private double radarDetectionProbability(double rangeM, double sizeM2, DrivingEnvironment env) {
        double rangeFactor = 1.0 - (rangeM / RADAR_MAX_RANGE_M) * 0.38;
        double referenceArea = 10.0;
        double sizeFactor = Math.sqrt(sizeM2 / referenceArea);
        sizeFactor = clamp(sizeFactor, 0.12, 1.0);
        return clamp(0.94 * rangeFactor * sizeFactor * env.radarWeatherFactor(), 0.0, 1.0);
    }

    private record RadarTrack(double rangeM, double relSpeedKph, long ts) {
        RadarReading toReading() {
            return new RadarReading(rangeM, relSpeedKph, ts);
        }
    }

    public CameraReading[] buildCameraReadings(WorldState world) {
        long ts = System.currentTimeMillis();
        DrivingEnvironment env = DrivingEnvironment.forSimTime(world.simTimeS());
        CarBlock ego = world.ego();
        Aabb e = ego.aabb();

        int max = 12;
        int total = world.npcs().size() + world.pedestrians().size();

        // Report a classification for each visible object; confidence decreases with distance.
        CameraReading[] tmp = new CameraReading[Math.min(max, total)];
        int k = 0;

        double missBase = 0.04 + (1.0 - env.ambientLight()) * 0.14 + (1.0 - env.cameraWeatherFactor()) * 0.12;

        for (CarBlock npc : world.npcs()) {
            if (k >= tmp.length) break;
            Aabb n = npc.aabb();

            double dy = e.y() - (n.y() + n.h());
            if (dy <= 0) continue;
            if (dy > 420) continue;

            if (rng.nextDouble() < missBase) continue;

            double conf = 1.0 - (dy / 420.0);
            conf = clamp01(conf * (0.70 + rng.nextDouble() * 0.30));
            conf *= env.ambientLight() * env.cameraWeatherFactor();
            conf = clamp01(conf);
            tmp[k++] = new CameraReading("car", Double.valueOf(conf), Long.valueOf(ts));
        }

        for (PedestrianBlock p : world.pedestrians()) {
            if (k >= tmp.length) break;
            Aabb n = p.aabb();

            double dy = e.y() - (n.y() + n.h());
            if (dy <= 0) continue;
            if (dy > 420) continue;

            double pedMiss = missBase + 0.05;
            if (rng.nextDouble() < pedMiss) continue;

            double conf = 1.0 - (dy / 420.0);
            conf = clamp01(conf * (0.65 + rng.nextDouble() * 0.35));
            conf *= env.ambientLight() * env.cameraWeatherFactor();
            conf = clamp01(conf);
            tmp[k++] = new CameraReading("pedestrian", Double.valueOf(conf), Long.valueOf(ts));
        }

        CameraReading[] out = new CameraReading[k];
        System.arraycopy(tmp, 0, out, 0, k);
        return out;
    }

    /**
     * Vehicle speed (km/h) used for wheel simulation, clamped to {@link #WHEEL_SPEED_MIN_KMH}–{@link #WHEEL_SPEED_MAX_KMH}.
     */
    public double simulatedWheelSpeedKmh(WorldState world) {
        double lateralKmh = world.egoSpeedPixelsPerSec() / pxPerMeter * 3.6;
        lateralKmh = clamp(lateralKmh, WHEEL_SPEED_MIN_KMH, WHEEL_SPEED_MAX_KMH);

        double cruiseKmh = 125.0 + 125.0 * Math.sin(world.simTimeS() * 0.11);
        cruiseKmh = clamp(cruiseKmh, WHEEL_SPEED_MIN_KMH, WHEEL_SPEED_MAX_KMH);

        return clamp(
                0.38 * lateralKmh + 0.62 * cruiseKmh,
                WHEEL_SPEED_MIN_KMH,
                WHEEL_SPEED_MAX_KMH
        );
    }

    public WheelSpeedReading[] buildWheelSpeedReadings(WorldState world) {
        long ts = System.currentTimeMillis();

        double t = world.simTimeS();
        double egoKmh = egoSpeedKmh(world);
        double decelKmhPerS = updateLongitudinalDeceleration(t, egoKmh);

        double vKmh = simulatedWheelSpeedKmh(world);
        double speedMps = vKmh / 3.6;

        // RPM = (linear speed / circumference) * 60
        double circumference = 2.0 * Math.PI * wheelRadiusM;
        double rpmBase = (speedMps / circumference) * 60.0;

        double sigmaBase = 5.0;
        double slipSigmaExtra = 0.0;
        double tractionIntensity = 0.0;
        if (decelKmhPerS >= WHEEL_RAPID_DECEL_THRESHOLD_KMH_S) {
            double span = WHEEL_SEVERE_DECEL_THRESHOLD_KMH_S - WHEEL_RAPID_DECEL_THRESHOLD_KMH_S;
            double u = clamp((decelKmhPerS - WHEEL_RAPID_DECEL_THRESHOLD_KMH_S) / Math.max(span, 1e-6), 0.0, 1.5);
            slipSigmaExtra = 10.0 + 28.0 * u;
            tractionIntensity = 18.0 + 52.0 * u;
            if (decelKmhPerS >= WHEEL_SEVERE_DECEL_THRESHOLD_KMH_S) {
                slipSigmaExtra += 16.0;
                tractionIntensity += 40.0;
            }
        }

        WheelSpeedReading[] wheels = new WheelSpeedReading[4];
        for (int i = 0; i < 4; i++) {
            double noise = rng.nextGaussian() * (sigmaBase + slipSigmaExtra);
            noise += wheelTractionBias(i, tractionIntensity);
            double noisy = Math.max(0.0, rpmBase + noise);
            wheels[i] = new WheelSpeedReading(Double.valueOf(noisy), Long.valueOf(ts));
        }
        return wheels;
    }

    private double egoSpeedKmh(WorldState world) {
        Vec2 v = world.ego().vel();
        double speedPxS = Math.hypot(v.x(), v.y());
        return speedPxS / pxPerMeter * 3.6;
    }

    /**
     * Uses simulation time steps so duplicate sensor polls at the same {@code simTimeS} do not corrupt dt.
     */
    private double updateLongitudinalDeceleration(double simTimeS, double egoKmh) {
        final double sameInstantEps = 1e-7;
        if (prevDecelSimTimeS >= 0.0 && Math.abs(simTimeS - prevDecelSimTimeS) < sameInstantEps) {
            return lastLongitudinalDecelKmhPerS;
        }
        if (prevDecelSimTimeS < 0.0) {
            prevEgoSpeedKmhForDecel = egoKmh;
            prevDecelSimTimeS = simTimeS;
            lastLongitudinalDecelKmhPerS = 0.0;
            return 0.0;
        }
        double dt = simTimeS - prevDecelSimTimeS;
        if (dt <= 1e-9) {
            prevEgoSpeedKmhForDecel = egoKmh;
            prevDecelSimTimeS = simTimeS;
            return lastLongitudinalDecelKmhPerS;
        }
        double drop = prevEgoSpeedKmhForDecel - egoKmh;
        double decel = (drop > 0.0) ? (drop / dt) : 0.0;
        lastLongitudinalDecelKmhPerS = decel;
        prevEgoSpeedKmhForDecel = egoKmh;
        prevDecelSimTimeS = simTimeS;
        return decel;
    }

    /** Correlated wheel differences under hard braking / slip (FL, FR, RL, RR pattern). */
    private double wheelTractionBias(int wheelIndex, double intensity) {
        if (intensity <= 1e-6) return 0.0;
        double[] w = { -1.0, 1.0, -0.78, 0.78 };
        return intensity * w[wheelIndex] * rng.nextGaussian();
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

