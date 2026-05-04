package core.src.core.Implementions;

import core.src.core.Interfaces.Sensors.SensorWheelSpeed;

public record WheelSpeedReading(Double rpm, Long timestamp) implements SensorWheelSpeed {

    @Override
    public Double wheelSpeed(int wheelIndex) {
        return rpm ;
    }

    @Override
    public Double rpm() {
        return rpm;

    }



    @Override
    public Long timeStamp() {
        return timestamp;
    }




}
