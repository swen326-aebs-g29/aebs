package core.src.main.java.Interfaces.Controls;

import core.src.main.java.Implementions.CameraReading;

import core.src.main.java.Implementions.RadarReading;
import core.src.main.java.Implementions.WheelSpeedReading;

public interface ISensorSystem {
   // RadarReading getRadarReading();
    RadarReading[] getRadarReadings();
  //  CameraReading getCameraReading();
    CameraReading[] getCameraReadings();

   // WheelSpeedReading getWheelSpeedReading();
    WheelSpeedReading[] getWheelSpeedReadings();
}
