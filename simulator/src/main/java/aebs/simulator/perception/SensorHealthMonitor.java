package aebs.simulator.perception;

import java.util.ArrayList;
import java.util.List;

/**
 * Lightweight fault handling: detect abnormal values, dropouts, and stale data.
 * Designed for the simulator (not a full ISO-26262 health monitor).
 */
public final class SensorHealthMonitor {
    public record Health(boolean ok, String summary) {}

    private double lastRadarOkS = -1.0;
    private double lastCameraOkS = -1.0;
    private double lastWheelOkS = -1.0;

    private static final double DROPOUT_S = 0.25; // allow brief empty windows

    public Health evaluate(double simTimeS, RadarReading[] radar, CameraReading[] camera, WheelSpeedReading[] wheel) {
        List<String> issues = new ArrayList<>();

        // Radar
        if (radar == null || radar.length == 0) {
            if (lastRadarOkS >= 0.0 && (simTimeS - lastRadarOkS) > DROPOUT_S) {
                issues.add("radar_dropout");
            }
        } else {
            boolean bad = false;
            for (RadarReading r : radar) {
                if (!isFinite(r.distanceMetres()) || r.distanceMetres() < SimulatedSensors.RADAR_MIN_RANGE_M || r.distanceMetres() > SimulatedSensors.RADAR_MAX_RANGE_M) {
                    bad = true;
                    break;
                }
                if (!isFinite(r.speedKph()) || Math.abs(r.speedKph()) > 400.0) {
                    bad = true;
                    break;
                }
            }
            if (bad) {
                issues.add("radar_abnormal");
            } else {
                lastRadarOkS = simTimeS;
            }
        }

        // Camera
        if (camera == null || camera.length == 0) {
            if (lastCameraOkS >= 0.0 && (simTimeS - lastCameraOkS) > DROPOUT_S) {
                issues.add("camera_dropout");
            }
        } else {
            boolean bad = false;
            for (CameraReading c : camera) {
                if (c.classification() == null || c.classification().isBlank()) { bad = true; break; }
                Double conf = c.confidence();
                if (conf != null && (!isFinite(conf) || conf < 0.0 || conf > 1.0)) { bad = true; break; }
            }
            if (bad) {
                issues.add("camera_abnormal");
            } else {
                lastCameraOkS = simTimeS;
            }
        }

        // Wheel
        if (wheel == null || wheel.length == 0) {
            if (lastWheelOkS >= 0.0 && (simTimeS - lastWheelOkS) > DROPOUT_S) {
                issues.add("wheel_dropout");
            }
        } else {
            boolean bad = false;
            for (WheelSpeedReading w : wheel) {
                Double rpm = w.rpm();
                if (rpm == null) continue;
                if (!isFinite(rpm) || rpm < 0.0 || rpm > 10_000.0) { bad = true; break; }
            }
            if (bad || wheel.length < 4) {
                issues.add("wheel_abnormal");
            } else {
                lastWheelOkS = simTimeS;
            }
        }

        if (issues.isEmpty()) return new Health(true, "OK");
        return new Health(false, String.join(",", issues));
    }

    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}

