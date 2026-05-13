package aebs.simulator.perception;

// Wheel Speed Data Container
public record WheelSpeedReading(Double rpm, Long timestamp)
        implements SensorWheelSpeed {
    @Override
    public Double RPM() {
        return rpm;
    }

    @Override
    public Double wheelSpeed(int wheelIndex) {
        return rpm;
    }

    @Override
    public Long timestamp() {
        return timestamp;
    }
}
