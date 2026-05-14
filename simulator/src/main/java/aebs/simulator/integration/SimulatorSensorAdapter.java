package aebs.simulator.integration;

import Implementions.WheelSpeedReading;
import nz.ac.vuw.swen326.aebs.core.CameraReading;
import nz.ac.vuw.swen326.aebs.core.RadarReading;
import nz.ac.vuw.swen326.aebs.core.SensorSnapshot;

public final class SimulatorSensorAdapter {
    public SensorSnapshot toCoreSnapshot(
            aebs.simulator.perception.RadarReading[] radarReadings,
            aebs.simulator.perception.CameraReading[] cameraReadings,
            aebs.simulator.perception.WheelSpeedReading[] wheelSpeedReadings,
            boolean sensorsHealthy
    ) {
        return new SensorSnapshot(
                toCoreRadar(radarReadings),
                toCoreCamera(cameraReadings),
                toCoreWheel(wheelSpeedReadings),
                sensorsHealthy,
                System.currentTimeMillis()
        );
    }

    public WheelSpeedReading[] toCoreWheel(aebs.simulator.perception.WheelSpeedReading[] wheelSpeedReadings) {
        if (wheelSpeedReadings == null) {
            return new WheelSpeedReading[0];
        }

        WheelSpeedReading[] translated = new WheelSpeedReading[wheelSpeedReadings.length];
        for (int i = 0; i < wheelSpeedReadings.length; i++) {
            aebs.simulator.perception.WheelSpeedReading reading = wheelSpeedReadings[i];
            if (reading == null) {
                translated[i] = null;
                continue;
            }

            double rpm = reading.rpm() == null ? 0.0 : reading.rpm();
            long timestamp = reading.timestamp() == null ? 0L : reading.timestamp();
            translated[i] = new WheelSpeedReading(rpm, timestamp);
        }
        return translated;
    }

    private RadarReading[] toCoreRadar(aebs.simulator.perception.RadarReading[] radarReadings) {
        if (radarReadings == null) {
            return new RadarReading[0];
        }

        RadarReading[] translated = new RadarReading[radarReadings.length];
        for (int i = 0; i < radarReadings.length; i++) {
            aebs.simulator.perception.RadarReading reading = radarReadings[i];
            if (reading == null) {
                translated[i] = null;
                continue;
            }

            translated[i] = new RadarReading(reading.distanceMetres(), reading.speedKph(), reading.timestamp());
        }
        return translated;
    }

    private CameraReading[] toCoreCamera(aebs.simulator.perception.CameraReading[] cameraReadings) {
        if (cameraReadings == null) {
            return new CameraReading[0];
        }

        CameraReading[] translated = new CameraReading[cameraReadings.length];
        for (int i = 0; i < cameraReadings.length; i++) {
            aebs.simulator.perception.CameraReading reading = cameraReadings[i];
            if (reading == null) {
                translated[i] = null;
                continue;
            }

            String classification = reading.classification() == null ? "" : reading.classification();
            double confidence = reading.confidence() == null ? 0.0 : reading.confidence();
            long timestamp = reading.timestamp() == null ? 0L : reading.timestamp();
            translated[i] = new CameraReading(classification, confidence, timestamp);
        }
        return translated;
    }
}
