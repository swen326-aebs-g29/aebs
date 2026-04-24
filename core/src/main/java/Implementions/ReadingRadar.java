package core.src.core.Implementions;

import core.src.core.Interfaces.Sensors.sensorReading;

public class ReadingRadar implements sensorReading {
    private final double distance;
    private final double speed;
    private final String cameraObject;
    public ReadingRadar(double distance, double speed, String cameraObject) {
        this.distance = distance;
        this.speed = speed;
        this.cameraObject = cameraObject;
    }
    public double getRadarReading(){
        return distance;
    }
    public String getCameraObject(){
        return cameraObject;

    }
    public double getWheelSpeed(){
        return speed;
    }
}
