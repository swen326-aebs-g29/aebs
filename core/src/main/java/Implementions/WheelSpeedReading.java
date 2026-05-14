package core.src.main.java.Implementions;

import core.src.main.java.Interfaces.Sensors.SensorWheelSpeed;

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
