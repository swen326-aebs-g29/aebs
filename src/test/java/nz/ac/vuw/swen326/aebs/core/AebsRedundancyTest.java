package nz.ac.vuw.swen326.aebs.core;

import aebs.simulator.perception.*;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for sensor redundancy and voting logic.
 * REQ-017, REQ-018
 */
class AebsRedundancyTest {

    // TEST CASE 017 - REQ-017
    @Test
    void testRedundantSensorFailover() {
        // PRE: sensor A fails (empty array)
        RadarReading[] sensorA = new RadarReading[0];
        RadarReading[] sensorB = {
                new RadarReading(50.0, 30.0, System.currentTimeMillis())
        };
        // STEPS: fuse with failed sensor A
        RadarReading[] result = RedundantSensorFusion.fuseRadar(sensorA, sensorB);
        // EXPECT: sensor B provides data without interruption
        assertNotNull(result);
        assertTrue(result.length > 0,
                "Fusion should fall back to sensor B when A fails");
    }

    // TEST CASE 018 - REQ-018
    @Test
    void testMultipleSensorInputsAccepted() {
        // PRE: both sensors active
        RadarReading[] sensorA = {
                new RadarReading(50.0, 30.0, System.currentTimeMillis())
        };
        RadarReading[] sensorB = {
                new RadarReading(80.0, 20.0, System.currentTimeMillis())
        };
        // STEPS: fuse both active sensors
        RadarReading[] result = RedundantSensorFusion.fuseRadar(sensorA, sensorB);
        // EXPECT: both inputs accepted by fusion logic
        assertNotNull(result);
        assertTrue(result.length > 0,
                "Fusion should accept inputs from both sensors");
    }
}