package core.src.main.java.Implementions;

import core.src.main.java.Interfaces.Controls.ISensorSystem;
import core.src.main.java.Implementions.RadarReading;
import core.src.main.java.Implementions.CameraReading;
import core.src.main.java.Implementions.WheelSpeedReading;

public class TimedSensorSystem implements ISensorSystem {

    private final ISensorSystem sensorSystem;

    private long lastRadar=0;
    private long lastCamera=0;
    private long lastTime=0;
    public TimedSensorSystem(ISensorSystem sensorSystem) {
        this.sensorSystem = sensorSystem;
    }
    @Override
    public RadarReading[] getRadarReadings() {
        waitFor(100, lastRadar);
        lastRadar+=100;
        return sensorSystem.getRadarReadings();
    }

    private void waitFor(long interval, long last) {
        long now = System.currentTimeMillis();
        long diff = now - last;

        if (diff < interval) {
            try {
                Thread.sleep(interval - diff);
            } catch (InterruptedException ignored) {}
        }
    }

    @Override
    public CameraReading[] getCameraReadings() {
        waitFor(50, lastCamera);
        lastCamera+=100;
        return sensorSystem.getCameraReadings();
    }

    @Override
    public WheelSpeedReading[] getWheelSpeedReadings() {
        waitFor(10, lastTime);
        lastTime+=100;
        return sensorSystem.getWheelSpeedReadings();
    }
}
