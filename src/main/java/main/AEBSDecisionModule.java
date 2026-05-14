package main;

import Implementions.ReadingRadar;

import java.util.Set;

public class AEBSDecisionModule {

    private static final double TTC_EMERGENCY = 1.5;
    private static final double TTC_HIGH = 3.0;
    private static final double TTC_MEDIUM = 5.0;

    private static final double MAX_DISTANCE = 50.0;
    private static final double MIN_DISTANCE = 1.0;
    private static final double MIN_CLOSING_SPEED = 0.5;

    private static final Set<String> HAZARD_OBJECTS = Set.of(
            "car", "truck", "pedestrian", "cyclist"
    );

    private double lastDistance = -1;
    private long lastTime = -1;

    public double decide(ReadingRadar reading, double sensitivity) {

        if (reading == null) return 0.0;

        double distance = reading.getRadarReading();
        String object = normalize(reading.getCameraObject());

        double closingSpeed = estimateClosingSpeed(distance);

        // Clamp sensitivity
        sensitivity = Math.max(0.5, Math.min(sensitivity, 2.0));

        // --- SANITY FILTER ---
        if (distance <= MIN_DISTANCE) return 0.0;
        if (closingSpeed <= MIN_CLOSING_SPEED) return 0.0;
        if (reading.getWheelSpeed() <= 0) return 0.0;
        if (distance > MAX_DISTANCE) return 0.0;

        // --- OBJECT FILTER ---
        if (!isHazard(object)) return 0.0;

        double ttc = calculateTTC(distance, closingSpeed);

        double emergency = TTC_EMERGENCY / sensitivity;
        double high = TTC_HIGH / sensitivity;
        double medium = TTC_MEDIUM / sensitivity;

        if (ttc <= emergency) return 1.0;
        if (ttc <= high) return 0.6;
        if (ttc <= medium) return 0.3;

        return 0.0;
    }

    // --- ESTIMATION LOGIC ---

    private double estimateClosingSpeed(double currentDistance) {

        long now = System.currentTimeMillis();

        if (lastDistance < 0 || lastTime < 0) {
            lastDistance = currentDistance;
            lastTime = now;
            return 0.0;
        }

        double deltaDistance = lastDistance - currentDistance;
        double deltaTime = (now - lastTime) / 1000.0;

        lastDistance = currentDistance;
        lastTime = now;

        if (deltaTime <= 0) return 0.0;

        return Math.max(0, deltaDistance / deltaTime);
    }

    // --- HELPERS ---

    private boolean isHazard(String object) {
        return HAZARD_OBJECTS.contains(object);
    }

    private String normalize(String obj) {
        return obj == null ? "" : obj.trim().toLowerCase();
    }

    private double calculateTTC(double distance, double closingSpeed) {
        if (closingSpeed <= 0) return Double.MAX_VALUE;
        return distance / closingSpeed;
    }
}