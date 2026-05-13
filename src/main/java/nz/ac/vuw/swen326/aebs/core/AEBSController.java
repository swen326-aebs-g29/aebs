package nz.ac.vuw.swen326.aebs.core;

import Implementions.WheelSpeedReading;
import main.BrakingControlModule;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.Supplier;

public final class AEBSController {
    private static final double EMERGENCY_DISTANCE_METRES = 12.0;
    private static final double BRAKE_DISTANCE_METRES = 28.0;
    private static final double BRAKE_TTC_SECONDS = 2.2;
    private static final double MIN_CAMERA_CONFIDENCE = 0.35;

    private final BrakingControlModule brakingControlModule;
    private final Supplier<WheelSpeedReading[]> wheelSupplier;

    public AEBSController(
            BrakingControlModule brakingControlModule,
            Supplier<WheelSpeedReading[]> wheelSupplier
    ) {
        this.brakingControlModule = Objects.requireNonNull(brakingControlModule, "brakingControlModule");
        this.wheelSupplier = Objects.requireNonNull(wheelSupplier, "wheelSupplier");
    }

    public ControllerDecision onSensorData(SensorSnapshot snapshot) {
        Objects.requireNonNull(snapshot, "snapshot");

        if (!snapshot.sensorsHealthy()) {
            brakingControlModule.stopBraking();
            return ControllerDecision.idle("sensor_fault");
        }

        if (snapshot.wheelSpeedReadings() == null || snapshot.wheelSpeedReadings().length == 0) {
            brakingControlModule.stopBraking();
            return ControllerDecision.idle("no_wheel_feedback");
        }

        RadarReading primaryThreat = findClosestRadarThreat(snapshot.radarReadings());
        if (primaryThreat == null) {
            brakingControlModule.stopBraking();
            return ControllerDecision.idle("clear_path");
        }

        double closingSpeedMps = Math.max(0.0, primaryThreat.relativeSpeedKph() / 3.6);
        double timeToCollisionS = closingSpeedMps > 1e-6
                ? primaryThreat.distanceMetres() / closingSpeedMps
                : Double.POSITIVE_INFINITY;
        boolean cameraConfirmed = hasCameraConfirmation(snapshot.cameraReadings());

        boolean shouldBrake = primaryThreat.distanceMetres() <= EMERGENCY_DISTANCE_METRES
                || (cameraConfirmed
                && (primaryThreat.distanceMetres() <= BRAKE_DISTANCE_METRES
                || timeToCollisionS <= BRAKE_TTC_SECONDS));

        if (!shouldBrake) {
            brakingControlModule.stopBraking();
            return ControllerDecision.idle("threat_below_threshold");
        }

        double brakeLevel = calculateBrakeLevel(primaryThreat.distanceMetres(), timeToCollisionS);
        brakingControlModule.startBraking(brakeLevel, wheelSupplier);
        return new ControllerDecision(true, brakeLevel, cameraConfirmed ? "camera_confirmed_threat" : "radar_emergency");
    }

    private RadarReading findClosestRadarThreat(RadarReading[] radarReadings) {
        if (radarReadings == null || radarReadings.length == 0) {
            return null;
        }

        return Arrays.stream(radarReadings)
                .filter(Objects::nonNull)
                .filter(reading -> reading.distanceMetres() > 0.0)
                .filter(reading -> reading.relativeSpeedKph() > 0.0)
                .min((left, right) -> Double.compare(left.distanceMetres(), right.distanceMetres()))
                .orElse(null);
    }

    private boolean hasCameraConfirmation(CameraReading[] cameraReadings) {
        if (cameraReadings == null || cameraReadings.length == 0) {
            return false;
        }

        return Arrays.stream(cameraReadings)
                .filter(Objects::nonNull)
                .anyMatch(reading -> reading.confidence() >= MIN_CAMERA_CONFIDENCE
                        && !reading.classification().isBlank());
    }

    private double calculateBrakeLevel(double distanceMetres, double timeToCollisionS) {
        double distanceUrgency = 1.0 - Math.min(distanceMetres, BRAKE_DISTANCE_METRES) / BRAKE_DISTANCE_METRES;
        double ttcUrgency = Double.isFinite(timeToCollisionS)
                ? 1.0 - Math.min(timeToCollisionS, BRAKE_TTC_SECONDS) / BRAKE_TTC_SECONDS
                : 0.0;
        double urgency = Math.max(distanceUrgency, ttcUrgency);
        return clamp(0.35 + urgency * 0.65, 0.0, 1.0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
