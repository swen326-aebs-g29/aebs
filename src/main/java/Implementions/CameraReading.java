package core.src.core.Implementions;

import core.src.core.Interfaces.Sensors.SensorCamera;

public record CameraReading(String Classification, Double Confidence, long Timestamp) implements SensorCamera {

    @Override
    public String classification() {
        return Classification;
    }

    @Override
    public Double confidence() {
        return Confidence;
    }

    @Override
    public Long timestamp() {
        return Timestamp;
    }


}
