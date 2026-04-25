package aebs.simulator.perception;

import java.util.Arrays;
import java.util.Comparator;

/**
 * Simple redundancy fusion: if one sensor chain fails (empty), fall back to the other.
 * If both provide data, combine and keep a small, deterministic subset.
 */
public final class RedundantSensorFusion {
    private RedundantSensorFusion() {}

    public static RadarReading[] fuseRadar(RadarReading[] a, RadarReading[] b) {
        if (a == null || a.length == 0) return (b == null) ? new RadarReading[0] : b;
        if (b == null || b.length == 0) return a;

        RadarReading[] out = new RadarReading[a.length + b.length];
        System.arraycopy(a, 0, out, 0, a.length);
        System.arraycopy(b, 0, out, a.length, b.length);
        Arrays.sort(out, Comparator.comparingDouble(RadarReading::distanceMetres));

        // Keep the closest two tracks (same behavior as a single chain).
        if (out.length <= 2) return out;
        return new RadarReading[]{out[0], out[1]};
    }

    public static CameraReading[] fuseCamera(CameraReading[] a, CameraReading[] b) {
        if (a == null || a.length == 0) return (b == null) ? new CameraReading[0] : b;
        if (b == null || b.length == 0) return a;

        CameraReading[] out = new CameraReading[a.length + b.length];
        System.arraycopy(a, 0, out, 0, a.length);
        System.arraycopy(b, 0, out, a.length, b.length);
        // Keep at most 12 detections total (same cap used in SimulatedSensors).
        if (out.length <= 12) return out;
        return Arrays.copyOf(out, 12);
    }

    public static WheelSpeedReading[] fuseWheel(WheelSpeedReading[] a, WheelSpeedReading[] b) {
        if (a == null || a.length == 0) return (b == null) ? new WheelSpeedReading[0] : b;
        if (b == null || b.length == 0) return a;

        // For wheel speed, keep A unless it looks invalid; otherwise fall back to B.
        // (We keep the shape as 4 wheel readings.)
        if (a.length >= 4) return a;
        if (b.length >= 4) return b;
        return a;
    }
}

