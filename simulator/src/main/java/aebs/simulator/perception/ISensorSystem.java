package aebs.simulator.perception;

public interface ISensorSystem {
    RadarReading[] getRadarReadings();

    CameraReading[] getCameraReadings();

    WheelSpeedReading[] getWheelSpeedReadings();
}
