package nz.ac.vuw.swen326.aebs.core;

import Implementions.WheelSpeedReading;

public record SensorSnapshot(
        RadarReading[] radarReadings,
        CameraReading[] cameraReadings,
        WheelSpeedReading[] wheelSpeedReadings,
        boolean sensorsHealthy,
        long timestamp
) {
}
