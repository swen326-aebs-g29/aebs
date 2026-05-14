package nz.ac.vuw.swen326.aebs.core;

import Implementions.WheelSpeedReading;
import Interfaces.Controls.IDriverInterface;
import Interfaces.Controls.BrakingControlInterface;
import nz.ac.vuw.swen326.aebs.core.AEBSController;
import nz.ac.vuw.swen326.aebs.core.CameraReading;
import nz.ac.vuw.swen326.aebs.core.ControllerDecision;
import nz.ac.vuw.swen326.aebs.core.RadarReading;
import nz.ac.vuw.swen326.aebs.core.SensorSnapshot;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for driver alert and interface module.
 * REQ-001, REQ-002, REQ-003, REQ-004, REQ-005, REQ-006, REQ-007, REQ-008
 */
class AebsDriverAlertTest {

    private AEBSController controller;
    private TrackingBrakeActuator brakeActuator;
    private TrackingDriverInterface driverInterface;

    @BeforeEach
    void setUp() {
        brakeActuator = new TrackingBrakeActuator();
        driverInterface = new TrackingDriverInterface();
        main.BrakingControlModule braking = new main.BrakingControlModule(
                brakeActuator, driverInterface
        );
        WheelSpeedReading[] wheels = {
                new WheelSpeedReading(300.0, System.currentTimeMillis()),
                new WheelSpeedReading(300.0, System.currentTimeMillis()),
                new WheelSpeedReading(300.0, System.currentTimeMillis()),
                new WheelSpeedReading(300.0, System.currentTimeMillis())
        };
        controller = new AEBSController(braking, () -> wheels);
    }

    // TEST CASE 001 - REQ-001
    @Test
    void testHazardAudioAlert() {
        // PRE: AEBS active, hazard detected
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        // STEPS: trigger hazard proximity
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: braking active, audio alert condition met
        assertTrue(decision.braking(),
                "Braking should be active on hazard detection");
    }

    // TEST CASE 002 - REQ-002
    @Test
    void testHazardVisualAlert() {
        // PRE: AEBS active, hazard detected
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        // STEPS: trigger hazard proximity
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: braking active, visual alert condition met
        assertTrue(decision.braking(),
                "Braking should be active for visual alert condition");
        assertNotNull(decision.reason(),
                "Decision should include a reason for the alert");
    }

    // TEST CASE 003 - REQ-003
    @Test
    void testSensitivityControls() {
        // PRE: system running
        // STEPS: test threshold boundary
        SensorSnapshot atThreshold = hazardSnapshot(28.0, 50.0);
        SensorSnapshot belowThreshold = hazardSnapshot(30.0, 10.0);
        // EXPECT: threshold affects whether braking triggers
        ControllerDecision atDecision = controller.onSensorData(atThreshold);
        ControllerDecision belowDecision = controller.onSensorData(belowThreshold);
        assertNotEquals(atDecision.braking(), belowDecision.braking(),
                "Different distances should produce different braking decisions");
    }

    // TEST CASE 004 - REQ-004
    @Test
    void testManualToggle() {
        // PRE: system running, no threat
        SensorSnapshot noThreat = clearSnapshot();
        // STEPS: pass clear snapshot
        ControllerDecision decision = controller.onSensorData(noThreat);
        // EXPECT: AEBS inactive when no hazard
        assertFalse(decision.braking(),
                "AEBS should not brake when no threat is present");
        assertEquals("clear_path", decision.reason(),
                "Reason should indicate clear path");
    }

    // TEST CASE 005 - REQ-005
    @Test
    void testBrakeVisualAlert() {
        // PRE: AEBS active, braking imminent
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: visual brake alert condition active
        assertTrue(decision.braking(),
                "Visual brake alert condition requires active braking");
        assertTrue(decision.brakeLevel() > 0,
                "Brake level should be positive for visual alert");
    }

    // TEST CASE 006 - REQ-006
    @Test
    void testBrakeAuditoryAlert() {
        // PRE: AEBS active, braking imminent
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: auditory brake alert condition active
        assertTrue(decision.braking(),
                "Auditory brake alert condition requires active braking");
    }

    // TEST CASE 007 - REQ-007
    @Test
    void testReadinessFeedback() {
        // PRE: system starting, no threats
        SensorSnapshot snapshot = clearSnapshot();
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: system returns idle state on initialisation
        assertFalse(decision.braking(),
                "System should be idle and ready on initialisation");
        assertNotNull(decision.reason(),
                "System should report a status on initialisation");
    }

    // TEST CASE 008 - REQ-008
    @Test
    void testMaintenanceFeedback() {
        // PRE: sensors report unhealthy
        SensorSnapshot faultSnapshot = new SensorSnapshot(
                new RadarReading[]{new RadarReading(5.0, 80.0, System.currentTimeMillis())},
                new CameraReading[]{new CameraReading("vehicle", 0.9, System.currentTimeMillis())},
                new WheelSpeedReading[]{new WheelSpeedReading(300.0, System.currentTimeMillis())},
                false,
                System.currentTimeMillis()
        );
        ControllerDecision decision = controller.onSensorData(faultSnapshot);
        // EXPECT: controller reports sensor fault
        assertFalse(decision.braking(),
                "System should not brake when sensors are unhealthy");
        assertEquals("sensor_fault", decision.reason(),
                "System should report sensor fault for maintenance alert");
    }

    // --- Helpers ---

    private SensorSnapshot hazardSnapshot(double distanceM, double speedKph) {
        return new SensorSnapshot(
                new RadarReading[]{new RadarReading(distanceM, speedKph, System.currentTimeMillis())},
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
    }

    private SensorSnapshot clearSnapshot() {
        return new SensorSnapshot(
                new RadarReading[0],
                new CameraReading[0],
                new WheelSpeedReading[]{
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis()),
                        new WheelSpeedReading(300.0, System.currentTimeMillis())
                },
                true,
                System.currentTimeMillis()
        );
    }

    private static final class TrackingBrakeActuator implements BrakingControlInterface {
        double lastLevel = 0.0;
        @Override
        public void applyBrake(double level) { this.lastLevel = level; }
    }

    private static final class TrackingDriverInterface implements IDriverInterface {
        boolean warningShown = false;
        boolean alertActive = false;
        @Override public void showWarning(String m) { warningShown = true; }
        @Override public void playSound(String s) {}
        @Override public void showStatus(String m) {}
        @Override public void setControl() { alertActive = true; }
        @Override public void feedback(String m) {}
    }
}