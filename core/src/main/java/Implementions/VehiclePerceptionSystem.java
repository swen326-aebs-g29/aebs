package core.src.main.java.Implementions;
import core.src.main.java.Interfaces.Controls.ISensorSystem;


public class VehiclePerceptionSystem implements ISensorSystem {

    private RadarReading[] radarData;
    private CameraReading[] cameraData;
    private WheelSpeedReading[] wheelData;

    public VehiclePerceptionSystem(RadarReading[] radars,
                                   CameraReading[] cameras,
                                   WheelSpeedReading[] wheels) {
        this.radarData = radars;
        this.cameraData = cameras;
        this.wheelData = wheels;
    }

    @Override
    public RadarReading[] getRadarReadings() {
        return radarData;
    }

    @Override
    public CameraReading[] getCameraReadings() {
        return cameraData;
    }

    @Override
    public WheelSpeedReading[] getWheelSpeedReadings() {
        return wheelData;
    }
    public void updateAllSensors(RadarReading[] r, CameraReading[] c, WheelSpeedReading[] w) {
        this.radarData=r;
        this.cameraData=c;
        this.wheelData=w;
    }
}
