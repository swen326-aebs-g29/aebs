package nz.ac.vuw.swen326.aebs.core;

import aebs.simulator.faults.PerceptionFaultInjector;
import aebs.simulator.perception.*;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for fault detection, tolerance, and fail-safe behaviour.
 * REQ-030, REQ-031, REQ-032
 */
class AebsFaultHandlingTest {

    private PerceptionFaultInjector injector;
    private SensorHealthMonitor monitor;

    @BeforeEach
    void setUp() {
        injector = new PerceptionFaultInjector(42L);
        monitor = new SensorHealthMonitor();
    }

    // TEST CASE 030 - REQ-030
    @Test
    void testFaultDetection() {
        // PRE: inject abnormal radar value (out of range distance)
        RadarReading[] abnormal = {
                new RadarReading(-999.0, 999.0, System.currentTimeMillis())
        };
        // STEPS: evaluate sensor health with abnormal reading
        SensorHealthMonitor.Health health = monitor.evaluate(1.0, abnormal, new CameraReading[0], new WheelSpeedReading[0]);
        // EXPECT: fault detected and reported
        assertFalse(health.ok(), "Health monitor should detect abnormal radar values");
        assertTrue(health.summary().contains("radar_abnormal"),
                "Summary should identify radar as abnormal");
    }

    // TEST CASE 031 - REQ-031
    @Test
    void testFaultTolerance() {
        // PRE: radar sensor fails during operation (dropout)
        RadarReading[] normal = {
                new RadarReading(50.0, 30.0, System.currentTimeMillis())
        };
        // Establish a healthy baseline first
        monitor.evaluate(0.0, normal, new CameraReading[0], new WheelSpeedReading[0]);

        // STEPS: apply fault injector dropout window (tS between 16.0 and 18.0)
        RadarReading[] faulted = injector.applyRadar(17.0, normal);

        // EXPECT: system continues to handle the reading gracefully
        // (dropout may return empty array; health monitor flags it but doesn't crash)
        assertNotNull(faulted, "Fault injector should return a non-null array even during dropout");
        SensorHealthMonitor.Health health = monitor.evaluate(17.5, faulted,
                new CameraReading[0], new WheelSpeedReading[0]);
        assertNotNull(health, "Health monitor should return a result even under fault conditions");
    }

    // TEST CASE 032 - REQ-032
    @Test
    void testFailSafeActivation() {
        // PRE: all sensors report abnormal values simultaneously
        RadarReading[] badRadar = {
                new RadarReading(-1.0, Double.NaN, System.currentTimeMillis())
        };
        CameraReading[] badCamera = {
                new CameraReading("", -0.5, System.currentTimeMillis())
        };
        WheelSpeedReading[] badWheel = {
                new WheelSpeedReading(99999.0, System.currentTimeMillis())
        };
        // STEPS: evaluate health with all sensors abnormal
        SensorHealthMonitor.Health health = monitor.evaluate(1.0, badRadar, badCamera, badWheel);
        // EXPECT: system reports unhealthy state (fail-safe condition detected)
        assertFalse(health.ok(), "System should report unhealthy when all sensors are abnormal");
        assertFalse(health.summary().equals("OK"),
                "Summary should not be OK under total sensor failure");
    }
}