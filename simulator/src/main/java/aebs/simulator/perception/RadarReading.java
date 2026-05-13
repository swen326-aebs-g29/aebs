package aebs.simulator.perception;

// Radar Data Container
public record RadarReading(double distanceMetres, double speedKph, long timestamp) implements SensorRadar {
    @Override
    public double distance() {
        return distanceMetres;
    }

    @Override
    public String speedObject() {
        return String.format("%.1f kph", speedKph);
    }
}
