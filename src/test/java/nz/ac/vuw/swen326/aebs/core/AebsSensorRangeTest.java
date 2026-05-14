package nz.ac.vuw.swen326.aebs.core;

import aebs.simulator.faults.PerceptionFaultInjector;
import aebs.simulator.perception.CameraReading;
import aebs.simulator.perception.RadarReading;
import aebs.simulator.perception.WheelSpeedReading;
import aebs.simulator.perception.SensorHealthMonitor;
import aebs.simulator.perception.SimulatedSensors;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for sensor value ranges and boundary conditions.
 * REQ-025, REQ-026, REQ-027, REQ-028, REQ-029
 */
class AebsSensorRangeTest {

    private SensorHealthMonitor monitor;
    private PerceptionFaultInjector injector;

    @BeforeEach
    void setUp() {
        monitor = new SensorHealthMonitor();
        injector = new PerceptionFaultInjector(42L);
    }

    // TEST CASE 025 - REQ-025
    @Test
    void testRadarDetectionRange() {
        // PRE: objects at various distances
        // STEPS: test at 0.5m, 100m, 200m and out-of-range 250m
        RadarReading valid05  = new RadarReading(0.5,   10.0, System.currentTimeMillis());
        RadarReading valid100 = new RadarReading(100.0, 10.0, System.currentTimeMillis());
        RadarReading valid200 = new RadarReading(200.0, 10.0, System.currentTimeMillis());
        RadarReading outRange = new RadarReading(250.0, 10.0, System.currentTimeMillis());

        // EXPECT: valid readings within 0.5-200m pass health check
        SensorHealthMonitor.Health h05  = monitor.evaluate(1.0, new RadarReading[]{valid05},  new CameraReading[0], new WheelSpeedReading[0]);
        SensorHealthMonitor.Health h100 = monitor.evaluate(1.1, new RadarReading[]{valid100}, new CameraReading[0], new WheelSpeedReading[0]);
        SensorHealthMonitor.Health h200 = monitor.evaluate(1.2, new RadarReading[]{valid200}, new CameraReading[0], new WheelSpeedReading[0]);
        SensorHealthMonitor.Health hOut = monitor.evaluate(1.3, new RadarReading[]{outRange}, new CameraReading[0], new WheelSpeedReading[0]);

        assertTrue(h05.ok(),   "0.5m should be within valid radar range");
        assertTrue(h100.ok(),  "100m should be within valid radar range");
        assertTrue(h200.ok(),  "200m should be within valid radar range");
        assertFalse(hOut.ok(), "250m should be outside valid radar range");
    }

    // TEST CASE 026 - REQ-026
    @Test
    void testRadarAccuracyThresholds() {
        // PRE: normal radar reading in range
        RadarReading[] normal = {new RadarReading(50.0, 30.0, System.currentTimeMillis())};

        // STEPS: apply fault injector bias window (tS 10-14 applies -1.5m bias)
        RadarReading[] biased = injector.applyRadar(12.0, normal);

        // EXPECT: thresholds applied, biased reading is still within valid range
        assertNotNull(biased);
        assertTrue(biased.length > 0, "Biased radar should still return readings");
        assertTrue(biased[0].distanceMetres() >= SimulatedSensors.RADAR_MIN_RANGE_M,
                "Biased distance should remain above minimum range");
    }

    // TEST CASE 027 - REQ-027
    @Test
    void testCameraAccuracyAcrossConditions() {
        // PRE: camera readings with varying confidence levels
        CameraReading highConf = new CameraReading("car", 0.95, System.currentTimeMillis());
        CameraReading lowConf  = new CameraReading("car", 0.30, System.currentTimeMillis());

        // STEPS: apply fault injector reduced confidence window (tS 8-10)
        CameraReading[] readings = {highConf};
        CameraReading[] degraded = injector.applyCamera(9.0, readings);

        // EXPECT: confidence varies per conditions, classification still returned
        assertNotNull(degraded);
        assertTrue(degraded.length > 0, "Camera should still return readings under degraded conditions");
        assertNotNull(degraded[0].classification(), "Classification should still be present");
        assertTrue(degraded[0].confidence() < highConf.confidence(),
                "Confidence should be reduced under fault injection");
    }

    // TEST CASE 028 - REQ-028
    @Test
    void testWheelSpeedRange() {
        // PRE: simulate valid and out-of-range wheel speeds
        WheelSpeedReading valid0   = new WheelSpeedReading(0.0,    System.currentTimeMillis());
        WheelSpeedReading valid125 = new WheelSpeedReading(500.0,  System.currentTimeMillis());
        WheelSpeedReading valid250 = new WheelSpeedReading(1000.0, System.currentTimeMillis());

        // STEPS: check RPM values are non-null and non-negative
        // EXPECT: valid range 0-250 km/h accepted (represented as RPM)
        assertNotNull(valid0.RPM(),   "0 km/h wheel reading should return RPM");
        assertNotNull(valid125.RPM(), "125 km/h wheel reading should return RPM");
        assertNotNull(valid250.RPM(), "250 km/h wheel reading should return RPM");
        assertTrue(valid0.RPM()   >= 0, "RPM should be non-negative");
        assertTrue(valid125.RPM() >= 0, "RPM should be non-negative");
        assertTrue(valid250.RPM() >= 0, "RPM should be non-negative");

        // Out-of-range: RPM above 10,000 should be flagged by health monitor
        WheelSpeedReading outRange = new WheelSpeedReading(99999.0, System.currentTimeMillis());
        SensorHealthMonitor health = new SensorHealthMonitor();
        WheelSpeedReading[] four = {outRange, outRange, outRange, outRange};
        SensorHealthMonitor.Health h = health.evaluate(1.0, new RadarReading[0], new CameraReading[0], four);
        assertFalse(h.ok(), "RPM above 10,000 should be flagged as abnormal");
    }

    // TEST CASE 029 - REQ-029
    @Test
    void testDecelerationThreshold() {
        // PRE: check deceleration thresholds defined in SimulatedSensors
        // STEPS: verify threshold constants match spec
        // EXPECT: rapid deceleration threshold triggers braking/traction concern
        assertTrue(SimulatedSensors.WHEEL_RAPID_DECEL_THRESHOLD_KMH_S > 0,
                "Rapid deceleration threshold should be positive");
        assertTrue(SimulatedSensors.WHEEL_SEVERE_DECEL_THRESHOLD_KMH_S >
                        SimulatedSensors.WHEEL_RAPID_DECEL_THRESHOLD_KMH_S,
                "Severe threshold should be higher than rapid threshold");
    }
}