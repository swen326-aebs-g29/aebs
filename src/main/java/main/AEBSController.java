package main;

import Interfaces.Controls.*;
import Implementions.*;

public class AEBSController implements IAEBSCorer {

    private final ISensorSystem sensors;
    private final BrakingControlInterface brake;
    private final IDriverInterface driver;

    private final RobustSensorFusionModule fusion;
    private final AEBSDecisionModule decision;
    private final BrakingControlModule actuator;

    private boolean brakingActive = false;

    public AEBSController(ISensorSystem sensors,
                          BrakingControlInterface brake,
                          IDriverInterface driver) {

        this.sensors = sensors;
        this.brake = brake;
        this.driver = driver;

        this.fusion = new RobustSensorFusionModule(sensors, driver);

        this.decision = new AEBSDecisionModule();

        this.actuator = new BrakingControlModule(brake, driver);
    }

    @Override
    public void update() {

        // --- AEBS OFF ---
        if (!driver.isAEBSActive()) {
            if (brakingActive) {
                actuator.stopBraking();
                brakingActive = false;
            }
            driver.showStatus("AEBS OFF");
            return;
        }

        // --- 1. SENSOR FUSION ---
        ReadingRadar fused = fusion.getFusedReading();

        // FIX 2: consistent validation message
        if (!isValid(fused)) {
            driver.showStatus("Invalid sensor data");
            return;
        }

        // --- 2. DECISION ---
        double brakeLevel = decision.decide(fused, driver.getSensitivity());

        // --- 3. ACTION ---
        if (brakeLevel > 0) {

            if (!brakingActive) {

                // Trigger alerts once
                driver.showWarning(
                        "Obstacle: " + fused.getCameraObject() +
                                " Distance: " + fused.getRadarReading()
                );

                driver.playSound("warning_beep");
                driver.showVisual();

                actuator.startBraking(
                        brakeLevel,
                        sensors.getWheelSpeedReadings()
                );

                brakingActive = true;
            }

        } else {

            if (brakingActive) {
                actuator.stopBraking();
                driver.showStatus("Hazard cleared");
                brakingActive = false;
            }
        }
    }

    // --- VALIDATION ---
    private boolean isValid(ReadingRadar fused) {

        if (fused == null) return false;

        if (Double.isNaN(fused.getRadarReading())) return false;

        if (fused.getRadarReading() < 0) return false;

        if (fused.getCameraObject() == null) return false;

        if (fused.getWheelSpeed() <= 0) return false;

        return true;
    }


    public BrakingControlModule getActuator() {
        return actuator;
    }
}