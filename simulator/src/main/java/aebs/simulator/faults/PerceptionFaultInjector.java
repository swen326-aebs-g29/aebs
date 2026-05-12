package aebs.simulator.faults;

import aebs.simulator.perception.CameraReading;
import aebs.simulator.perception.RadarReading;
import aebs.simulator.perception.WheelSpeedReading;

import java.util.Random;

/**
 * Simple fault injector that can corrupt arrays of readings.
 * Keeps types exactly as your record containers.
 */
public final class PerceptionFaultInjector {
    private final Random rng;

    public PerceptionFaultInjector(long seed) {
        this.rng = new Random(seed);
    }

    public RadarReading[] applyRadar(double tS, RadarReading[] in) {
        RadarReading[] out = in;

        // Example windows: noise burst, bias, dropout
        if (tS >= 4.0 && tS <= 9.0) { // noise burst
            out = mapRadar(out, r -> new RadarReading(
                    Math.max(0.0, r.distanceMetres() + rng.nextGaussian() * 2.0),
                    r.speedKph() + rng.nextGaussian() * 4.0,
                    r.timestamp()
            ));
        }
        if (tS >= 10.0 && tS <= 14.0) { // bias
            out = mapRadar(out, r -> new RadarReading(
                    Math.max(0.0, r.distanceMetres() - 1.5),
                    r.speedKph(),
                    r.timestamp()
            ));
        }
        if (tS >= 16.0 && tS <= 18.0) { // dropout
            if (rng.nextDouble() < 0.35) out = new RadarReading[0];
        }

        return out;
    }

    public CameraReading[] applyCamera(double tS, CameraReading[] in) {
        CameraReading[] out = in;
        if (tS >= 8.0 && tS <= 10.0) { // reduced confidence
            out = mapCamera(out, c -> new CameraReading(
                    c.classification(),
                    Double.valueOf(clamp01((c.confidence() == null ? 0.0 : c.confidence()) * 0.55)),
                    c.timestamp()));
        }
        if (tS >= 12.0 && tS <= 13.5) { // misclassification burst
            out = mapCamera(out, c -> new CameraReading(rng.nextDouble() < 0.4 ? "unknown" : c.classification(), c.confidence(), c.timestamp()));
        }
        return out;
    }

    public WheelSpeedReading[] applyWheel(double tS, WheelSpeedReading[] in) {
        WheelSpeedReading[] out = in;
        if (tS >= 6.0 && tS <= 7.0) { // spike RPM briefly
            out = mapWheel(out, w -> new WheelSpeedReading(
                    Double.valueOf((w.rpm() == null ? 0.0 : w.rpm()) + 180.0),
                    w.timestamp()));
        }
        if (tS >= 14.0 && tS <= 15.0) { // one wheel dropout-ish (stuck low)
            if (out.length >= 1) {
                WheelSpeedReading[] copy = out.clone();
                double r0 = copy[0].rpm() == null ? 0.0 : copy[0].rpm();
                copy[0] = new WheelSpeedReading(Double.valueOf(Math.max(0.0, r0 * 0.1)), copy[0].timestamp());
                out = copy;
            }
        }
        return out;
    }

    private interface RadarFn { RadarReading apply(RadarReading r); }
    private interface CameraFn { CameraReading apply(CameraReading c); }
    private interface WheelFn { WheelSpeedReading apply(WheelSpeedReading w); }

    private static RadarReading[] mapRadar(RadarReading[] in, RadarFn fn) {
        RadarReading[] out = new RadarReading[in.length];
        for (int i = 0; i < in.length; i++) out[i] = fn.apply(in[i]);
        return out;
    }

    private static CameraReading[] mapCamera(CameraReading[] in, CameraFn fn) {
        CameraReading[] out = new CameraReading[in.length];
        for (int i = 0; i < in.length; i++) out[i] = fn.apply(in[i]);
        return out;
    }

    private static WheelSpeedReading[] mapWheel(WheelSpeedReading[] in, WheelFn fn) {
        WheelSpeedReading[] out = new WheelSpeedReading[in.length];
        for (int i = 0; i < in.length; i++) out[i] = fn.apply(in[i]);
        return out;
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}

