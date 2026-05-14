package main;

import Interfaces.Controls.IDriverInterface;
import Interfaces.Controls.ISensorSystem;
import Implementions.*;

import java.util.*;
import java.util.stream.Collectors;

public class RobustSensorFusionModule {

    private final ISensorSystem sensors;
    private final IDriverInterface driver;



    public RobustSensorFusionModule(ISensorSystem sensors,
                                    IDriverInterface driver) {
        this.sensors = sensors;
        this.driver = driver;
    }

    public ReadingRadar getFusedReading() {

        RadarReading[] radars = sensors.getRadarReadings();
        CameraReading[] cameras = sensors.getCameraReadings();
        WheelSpeedReading[] wheels = sensors.getWheelSpeedReadings();

        double distance = fuseRadarDistance(radars);
        double speed = fuseRadarSpeed(radars);
        String object = fuseCamera(cameras);

        // Wheel speed is optional here but useful fallback
        double wheelSpeed = fuseWheelSpeed(wheels);

        // fallback if radar speed unreliable
        if (speed <= 0 && wheelSpeed > 0) {
            speed = wheelSpeed;
        }

        return new ReadingRadar(distance, speed, object);
    }

    // --- RADAR FUSION (MEDIAN VOTING) ---
    private double fuseRadarDistance(RadarReading[] radars) {

        List<Double> valid = Arrays.stream(radars)
                .filter(r -> r != null && r.valid())
                .map(RadarReading::distance)
                .sorted()
                .collect(Collectors.toList());

        if (valid.isEmpty()) {
            driver.notifyMaintenance("All radar sensors failed");
            return -1;
        }

        if (valid.size() == 1) {
            driver.notifyMaintenance("Radar redundancy degraded");
            return valid.get(0);
        }

        // Median voting (REQ-FUS-002)
        return valid.get(valid.size() / 2);
    }

    private double fuseRadarSpeed(RadarReading[] radars) {

        List<Double> speeds = Arrays.stream(radars)
                .filter(Objects::nonNull)
                .map(RadarReading::relativeSpeedKph)
                .collect(Collectors.toList());

        if (speeds.isEmpty()) return 0;

        return speeds.stream().mapToDouble(Double::doubleValue).average().orElse(0);
    }

    // --- CAMERA FUSION (MAJORITY + CONFIDENCE) ---
    private String fuseCamera(CameraReading[] cameras) {

        if (cameras == null || cameras.length == 0) {
            driver.notifyMaintenance("Camera system unavailable");
            return "unknown";
        }

        Map<String, Double> weightedVotes = new HashMap<>();

        for (CameraReading c : cameras) {
            if (c == null) continue;

            weightedVotes.put(
                    c.classification(),
                    weightedVotes.getOrDefault(c.classification(), 0.0)
                            + c.confidence()
            );
        }

        if (weightedVotes.isEmpty()) {
            driver.notifyMaintenance("Camera data invalid");
            return "unknown";
        }

        // Weighted voting (REQ-FUS-002)
        return Collections.max(
                weightedVotes.entrySet(),
                Map.Entry.comparingByValue()
        ).getKey();
    }

    // --- WHEEL SPEED FUSION ---
    private double fuseWheelSpeed(WheelSpeedReading[] wheels) {

        if (wheels == null || wheels.length == 0) {
            driver.notifyMaintenance("Wheel speed sensors unavailable");
            return 0;
        }

        return Arrays.stream(wheels)
                .filter(Objects::nonNull)
                .mapToDouble(WheelSpeedReading::rpm)
                .average()
                .orElse(0);
    }
}