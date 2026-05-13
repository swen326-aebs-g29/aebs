package aebs.simulator.environment;

/**
 * Synthetic light and weather used to modulate camera and radar behaviour.
 */
public final class DrivingEnvironment {
    private final double ambientLight;
    private final double radarWeatherFactor;
    private final double cameraWeatherFactor;

    private DrivingEnvironment(double ambientLight, double radarWeatherFactor, double cameraWeatherFactor) {
        this.ambientLight = ambientLight;
        this.radarWeatherFactor = radarWeatherFactor;
        this.cameraWeatherFactor = cameraWeatherFactor;
    }

    /**
     * @param simTimeS simulation clock in seconds
     */
    public static DrivingEnvironment forSimTime(double simTimeS) {
        double dayNight = 0.5 + 0.5 * Math.cos(simTimeS * 0.08);
        double ambient = 0.18 + 0.82 * dayNight;

        double cycle = (simTimeS * 0.045) % 1.0;
        double rain = Math.max(0.0, 1.0 - Math.abs(cycle - 0.25) * 5.0);
        rain = Math.min(1.0, rain);
        if (cycle < 0.05 || cycle > 0.45) rain *= 0.35;

        double fog = Math.max(0.0, 1.0 - Math.abs(cycle - 0.72) * 6.0);
        fog = Math.min(1.0, fog);
        if (cycle < 0.55 || cycle > 0.88) fog *= 0.25;

        double radarF = 1.0 - 0.42 * rain - 0.58 * fog;
        double camF = 1.0 - 0.32 * rain - 0.48 * fog - (1.0 - ambient) * 0.15;

        radarF = Math.max(0.12, Math.min(1.0, radarF));
        camF = Math.max(0.1, Math.min(1.0, camF));
        ambient = Math.max(0.12, Math.min(1.0, ambient));

        return new DrivingEnvironment(ambient, radarF, camF);
    }

    /** 0 (dark) .. 1 (bright) */
    public double ambientLight() {
        return ambientLight;
    }

    /** 1 clear, lower in rain/fog — radar detection strength */
    public double radarWeatherFactor() {
        return radarWeatherFactor;
    }

    /** 1 clear, lower in rain/fog/dim light — camera classification strength */
    public double cameraWeatherFactor() {
        return cameraWeatherFactor;
    }
}
