package core.src.main.java.Implementions;

import core.src.main.java.Interfaces.Sensors.SensorRadar;

public record RadarReading(double distance, double speedKph, long timestamp) implements SensorRadar {


    @Override
    public double distanceMetres() {
        return distance;
    }

    @Override
    public String speedObject() {
        return String.format("%.1f", speedKph);
    }
    public boolean valid() {
        return distance > 0;
    }

    public double relativeSpeedKph() {
        return speedKph;
    }

    @Override
    public long timestamp() {
        return timestamp;
    }






}
