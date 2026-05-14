package core.src.main.java.main;
import core.src.main.java.Interfaces.Controls.BrakingControlInterface;
import core.src.main.java.Interfaces.Controls.IDriverInterface;
import core.src.main.java.Interfaces.Controls.ISensorSystem;
import core.src.core.main.IAEBSCorer;
import core.src.main.java.Implementions.RadarReading;
import core.src.main.java.Implementions.CameraReading;
import core.src.main.java.Implementions.WheelSpeedReading;

import java.util.*;

public class AEBSController implements IAEBSCorer  {

    private final ISensorSystem sensors;
    private final BrakingControlInterface brake;
    private final IDriverInterface driver;

    public AEBSController(ISensorSystem sensors,
                          BrakingControlInterface brake,
                          IDriverInterface driver) {
        this.sensors = sensors;
        this.brake = brake;
        this.driver = driver;
    }

    public void update() {

        if (!driver.isAEBSActive()) {
            driver.showStatus("AEBS OFF");
            return;
        }

        // 1. Get sensor data
        RadarReading[] radar = sensors.getRadarReadings();
        CameraReading[] camera = sensors.getCameraReadings();
        WheelSpeedReading[] wheels = sensors.getWheelSpeedReadings();

        // 2. Redundancy check
        if (radar.length < 2) {
            driver.notifyMaintenance("Radar redundancy lost");
        }

        // 3. Voting (median distance)
        double distance = getMedianDistance(radar);
        double speed = getAverageSpeed(radar);

        // 4. Object detection
        String object = getDominantObject(camera);

        // 5. Risk calculation (TTC)
        double ttc = calculateTTC(distance, speed);

        // 6. Decide braking level
        double brakeLevel = decideBrakeLevel(ttc);

        // 7. Driver alerts
        if (brakeLevel > 0) {
            driver.showWarning("Obstacle: " + object + " TTC: " + String.format("%.2f", ttc));
            driver.playSound("beep");
        }

        // 8. Apply braking with retry logic
        if (brakeLevel > 0) {
            applyBrakingWithRetry(brakeLevel, wheels);
        }
    }


    private double getMedianDistance(RadarReading[] radars) {
        List<Double> distances = new ArrayList<>();

        for (RadarReading r : radars) {
            if (r.valid()) {
                distances.add(r.distanceMetres());
            }
        }

        if (distances.isEmpty()) return -1;

        Collections.sort(distances);
        return distances.get(distances.size() / 2);
    }

    private double getAverageSpeed(RadarReading[] radars) {
        double sum = 0;
        int count = 0;

        for (RadarReading r : radars) {
            sum += r.relativeSpeedKph();
            count++;
        }

        return count > 0 ? sum / count : 0;
    }

    private String getDominantObject(CameraReading[] cameras) {
        Map<String, Integer> count = new HashMap<>();

        for (CameraReading c : cameras) {
            count.put(c.classification(),
                    count.getOrDefault(c.classification(), 0) + 1);
        }

        return Collections.max(count.entrySet(), Map.Entry.comparingByValue()).getKey();
    }

    private double calculateTTC(double distance, double speedKph) {
        double speedMps = speedKph / 3.6;
        if (speedMps <= 0) return Double.MAX_VALUE;
        return distance / speedMps;
    }

    private double decideBrakeLevel(double ttc) {
        if (ttc < 1.5) return 1.0;
        if (ttc < 3.0) return 0.6;
        if (ttc < 5.0) return 0.3;
        return 0.0;
    }

    private void applyBrakingWithRetry(double level, WheelSpeedReading[] wheels) {

        int attempts = 0;

        while (attempts < 3) {

            brake.applyBrake(level);

            sleep(50); // REQ-020

            if (verifyBraking(wheels)) {
                driver.showStatus("Braking successful");
                return;
            }

            attempts++;
        }

        driver.showWarning("Brake failure!");
        driver.playSound("ALERT");
    }

    private boolean verifyBraking(WheelSpeedReading[] wheels) {
        double avg = Arrays.stream(wheels)
                .mapToDouble(WheelSpeedReading::rpm)
                .average()
                .orElse(0);

        return avg < 1000; // simple threshold (replace with proper curve later)
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {}
    }
}