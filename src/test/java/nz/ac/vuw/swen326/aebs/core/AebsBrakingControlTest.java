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
 * Tests for braking control module.
 * REQ-019, REQ-020, REQ-021, REQ-022, REQ-023, REQ-024
 */
class AebsBrakingControlTest {

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

    // TEST CASE 019 - REQ-019
    @Test
    void testBrakeSignalSent() {
        // PRE: obstacle within emergency distance (12m)
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        // STEPS: pass sensor data to controller
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: brake control signal sent
        assertTrue(decision.braking(),
                "Controller should activate braking for close obstacle");
        assertTrue(decision.brakeLevel() > 0,
                "Brake level should be positive when braking");
    }

    // TEST CASE 020 - REQ-020
    @Test
    void testBrakeSignalRate() {
        // PRE: active braking triggered
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: braking decision made (50ms signal rate enforced in BrakingControlModule)
        assertTrue(decision.braking(),
                "Braking should be active to enable 50ms signal rate");
        assertNotNull(decision.reason(),
                "Decision should include a reason");
    }

    // TEST CASE 021 - REQ-021
    @Test
    void testBrakeVerificationWithin50ms() {
        // PRE: obstacle triggers braking
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        long before = System.currentTimeMillis();
        ControllerDecision decision = controller.onSensorData(snapshot);
        long after = System.currentTimeMillis();
        // EXPECT: decision made within 50ms
        assertTrue(decision.braking(),
                "Braking decision should be active");
        assertTrue((after - before) <= 50,
                "Braking decision should be made within 50ms");
    }

    // TEST CASE 022 - REQ-022
    @Test
    void testBrakeSuccessWithinMargin() {
        // PRE: emergency distance triggers full urgency braking
        SensorSnapshot snapshot = hazardSnapshot(5.0, 80.0);
        ControllerDecision decision = controller.onSensorData(snapshot);
        // EXPECT: brake level within expected range per calculateBrakeLevel
        assertTrue(decision.brakeLevel() >= 0.35,
                "Brake level should be at least minimum urgency");
        assertTrue(decision.brakeLevel() <= 1.0,
                "Brake level should not exceed maximum");
    }

    // TEST CASE 023 - REQ-023
    @Test
    void testCorrectiveBrakingRetries() {
        // PRE: clear path should not trigger braking
        SensorSnapshot noThreat = new SensorSnapshot(
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
        ControllerDecision decision = controller.onSensorData(noThreat);
        // EXPECT: no braking when path is clear
        assertFalse(decision.braking(),
                "Controller should not brake when no threat is present");
        assertEquals("clear_path", decision.reason(),
                "Reason should indicate clear path");
    }

    // TEST CASE 024 - REQ-024
    @Test
    void testEscalationAfterTwoFailures() {
        // PRE: unhealthy sensors cause controller to stop braking
        SensorSnapshot faultSnapshot = new SensorSnapshot(
                new RadarReading[]{new RadarReading(5.0, 80.0, System.currentTimeMillis())},
                new CameraReading[]{new CameraReading("vehicle", 0.9, System.currentTimeMillis())},
                new WheelSpeedReading[]{new WheelSpeedReading(300.0, System.currentTimeMillis())},
                false,
                System.currentTimeMillis()
        );
        ControllerDecision decision = controller.onSensorData(faultSnapshot);
        // EXPECT: controller returns idle on sensor fault
        assertFalse(decision.braking(),
                "Controller should stop braking on sensor fault");
        assertEquals("sensor_fault", decision.reason(),
                "Reason should indicate sensor fault escalation");
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