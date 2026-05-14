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
 * Tests for fault detection, tolerance, and fail-safe behaviour.
 * REQ-030, REQ-031, REQ-032
 */
class AebsFaultHandlingTest {

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

    // TEST CASE 030 - REQ-030
    @Test
    void testFaultDetection() {
        // PRE: sensors report unhealthy
        SensorSnapshot faultSnapshot = new SensorSnapshot(
                new RadarReading[]{new RadarReading(-999.0, 999.0, System.currentTimeMillis())},
                new CameraReading[0],
                new WheelSpeedReading[]{new WheelSpeedReading(300.0, System.currentTimeMillis())},
                false,
                System.currentTimeMillis()
        );
        // STEPS: pass unhealthy snapshot to controller
        ControllerDecision decision = controller.onSensorData(faultSnapshot);
        // EXPECT: fault detected and reported
        assertFalse(decision.braking(),
                "Controller should not brake when sensors report fault");
        assertEquals("sensor_fault", decision.reason(),
                "Controller should report sensor fault");
    }

    // TEST CASE 031 - REQ-031
    @Test
    void testFaultTolerance() {
        // PRE: first call healthy, second call sensor fails
        SensorSnapshot healthy = new SensorSnapshot(
                new RadarReading[]{new RadarReading(50.0, 30.0, System.currentTimeMillis())},
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
        SensorSnapshot faulted = new SensorSnapshot(
                new RadarReading[]{new RadarReading(50.0, 30.0, System.currentTimeMillis())},
                new CameraReading[0],
                new WheelSpeedReading[]{new WheelSpeedReading(300.0, System.currentTimeMillis())},
                false,
                System.currentTimeMillis()
        );
        // STEPS: healthy call followed by faulted call
        controller.onSensorData(healthy);
        ControllerDecision faultDecision = controller.onSensorData(faulted);
        // EXPECT: system handles fault gracefully without crashing
        assertNotNull(faultDecision,
                "Controller should return a decision even under fault conditions");
        assertEquals("sensor_fault", faultDecision.reason(),
                "Controller should report sensor fault during fault tolerance check");
    }

    // TEST CASE 032 - REQ-032
    @Test
    void testFailSafeActivation() {
        // PRE: all sensors unhealthy simultaneously
        SensorSnapshot criticalFault = new SensorSnapshot(
                new RadarReading[0],
                new CameraReading[0],
                new WheelSpeedReading[0],
                false,
                System.currentTimeMillis()
        );
        // STEPS: trigger critical failure
        ControllerDecision decision = controller.onSensorData(criticalFault);
        // EXPECT: fail-safe mode activated (braking stopped, fault reported)
        assertFalse(decision.braking(),
                "System should not brake in fail-safe state");
        assertNotEquals("OK", decision.reason(),
                "System should not report OK under critical failure");
    }

    private static final class TrackingBrakeActuator implements BrakingControlInterface {
        double lastLevel = 0.0;
        @Override
        public void applyBrake(double level) { this.lastLevel = level; }
    }

    private static final class TrackingDriverInterface implements IDriverInterface {
        boolean warningShown = false;
        @Override public void showWarning(String m) { warningShown = true; }
        @Override public void playSound(String s) {}
        @Override public void showStatus(String m) {}
        @Override public void setControl() {}
        @Override public void feedback(String m) {}
    }
}