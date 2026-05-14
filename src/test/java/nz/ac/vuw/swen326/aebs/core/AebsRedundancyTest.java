package nz.ac.vuw.swen326.aebs.core;

import Implementions.WheelSpeedReading;
import Interfaces.Controls.BrakingControlInterface;
import Interfaces.Controls.IDriverInterface;
import nz.ac.vuw.swen326.aebs.core.AEBSController;
import nz.ac.vuw.swen326.aebs.core.CameraReading;
import nz.ac.vuw.swen326.aebs.core.ControllerDecision;
import nz.ac.vuw.swen326.aebs.core.RadarReading;
import nz.ac.vuw.swen326.aebs.core.SensorSnapshot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for sensor redundancy and voting logic.
 * REQ-017, REQ-018
 */
class AebsRedundancyTest {

    private AEBSController controller;

    @BeforeEach
    void setUp() {
        main.BrakingControlModule braking = new main.BrakingControlModule(
                new TrackingBrakeActuator(), new TrackingDriverInterface()
        );
        WheelSpeedReading[] wheels = {
                new WheelSpeedReading(300.0, System.currentTimeMillis()),
                new WheelSpeedReading(300.0, System.currentTimeMillis()),
                new WheelSpeedReading(300.0, System.currentTimeMillis()),
                new WheelSpeedReading(300.0, System.currentTimeMillis())
        };
        controller = new AEBSController(braking, () -> wheels);
    }

    // TEST CASE 017 - REQ-017
    @Test
    void testRedundantSensorFailover() {
        // PRE: primary radar fails (empty), secondary radar provides data
        // STEPS: pass snapshot with only camera confirmation, no radar
        SensorSnapshot noRadar = new SensorSnapshot(
                new RadarReading[0],
                new CameraReading[]{new CameraReading("vehicle", 0.9, System.currentTimeMillis())},
                new WheelSpeedReading[]{
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis())
                },
                true,
                System.currentTimeMillis()
        );
        ControllerDecision decision = controller.onSensorData(noRadar);
        // EXPECT: system handles missing radar gracefully without crashing
        assertNotNull(decision,
                "Controller should return a decision even with no radar data");
        assertEquals("clear_path", decision.reason(),
                "Controller should report clear path when radar fails with no threat confirmed");
    }

    // TEST CASE 018 - REQ-018
    @Test
    void testMultipleSensorInputsAccepted() {
        // PRE: both radar and camera active
        SensorSnapshot bothActive = new SensorSnapshot(
                new RadarReading[]{
                        new RadarReading(5.0, 80.0, System.currentTimeMillis()),
                        new RadarReading(8.0, 60.0, System.currentTimeMillis())
                },
                new CameraReading[]{new CameraReading("vehicle", 0.9, System.currentTimeMillis())},
                new WheelSpeedReading[]{
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis())
                },
                true,
                System.currentTimeMillis()
        );
        // STEPS: fuse both inputs
        ControllerDecision decision = controller.onSensorData(bothActive);
        // EXPECT: both inputs accepted, braking triggered on closest threat
        assertTrue(decision.braking(),
                "Controller should accept multiple sensor inputs and brake on closest threat");
    }

    private static final class TrackingBrakeActuator implements BrakingControlInterface {
        @Override public void applyBrake(double level) {}
    }

    private static final class TrackingDriverInterface implements IDriverInterface {
        @Override public void showWarning(String m) {}
        @Override public void playSound(String s) {}
        @Override public void showStatus(String m) {}
        @Override public void setControl() {}
        @Override public void feedback(String m) {}
    }
}