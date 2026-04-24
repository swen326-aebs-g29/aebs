package core.src.main.java.Implementions;

import core.src.main.java.Interfaces.Sensors.SensorCamera;

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
