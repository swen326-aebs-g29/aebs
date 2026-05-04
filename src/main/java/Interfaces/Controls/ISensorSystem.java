package core.src.core.Interfaces.Controls;

import core.src.core.Implementions.CameraReading;
import core.src.core.Implementions.RadarReading;
import core.src.core.Implementions.WheelSpeedReading;

public interface ISensorSystem {
   // RadarReading getRadarReading();
    RadarReading[] getRadarReadings();
  //  CameraReading getCameraReading();
    CameraReading[] getCameraReadings();

   // WheelSpeedReading getWheelSpeedReading();
    WheelSpeedReading[] getWheelSpeedReadings();
}
