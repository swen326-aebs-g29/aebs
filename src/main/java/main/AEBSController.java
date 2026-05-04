package core.src.core.main;

import core.src.core.Interfaces.Controls.*;
import core.src.core.Implementions.*;
//import core.src.core.*;

public class AEBSController implements IAEBSCorer {

    private final ISensorSystem sensors;
    private final BrakingControlInterface brake;
    private final IDriverInterface driver;

    private final RobustSensorFusionModule fusion;
    private final AEBSDecisionModule decision;
    private final BrakingActuatorModule actuator;

    public AEBSController(ISensorSystem sensors,
                          BrakingControlInterface brake,
                          IDriverInterface driver) {

        this.sensors = sensors;
        this.brake = brake;
        this.driver = driver;

        this.fusion = new RobustSensorFusionModule(sensors, driver);
        this.decision = new AEBSDecisionModule();
        this.actuator = new BrakingActuatorModule(brake, driver);
    }

    @Override
    public void update() {

        if (!driver.isAEBSActive()) {
            driver.showStatus("AEBS OFF");
            return;
        }

        // --- 1. FUSE SENSOR DATA ---
        ReadingRadar fused = fusion.getFusedReading();

        // Safety guard
        if (fused.getRadarReading() < 0) {
            driver.showStatus("Invalid sensor data");
            return;
        }

        // --- 2. DECISION LOGIC ---
        double brakeLevel = decision.decide(fused, driver.getSensitivity());

        // --- 3. DRIVER ALERT ---
        if (brakeLevel > 0) {
            driver.showWarning(
                    "Obstacle: " + fused.getCameraObject() +
                            " Distance: " + fused.getRadarReading()
            );
            driver.playSound("beep");
            driver.showVisual();
        }

        // --- 4. ACTUATION ---
        if (brakeLevel > 0) {
            actuator.executeBraking(
                    brakeLevel,
                    sensors.getWheelSpeedReadings()
            );
        }
    }
}