package aebs.simulator.perception;

// Camera Data Container
public record CameraReading(String classification, Double confidence, Long timestamp)
        implements SensorCamera {
    @Override
    public String classification() {
        return classification;
    }

    @Override
    public Double confidence() {
        return confidence;
    }

    @Override
    public Long timestamp() {
        return timestamp;
    }
}
