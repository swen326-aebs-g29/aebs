package nz.ac.vuw.swen326.aebs.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for sensor value ranges and boundary conditions.
 * REQ-025, REQ-026, REQ-027, REQ-028, REQ-029
 */
class AebsSensorRangeTest {

    // TEST CASE 025 - REQ-025
    @Test
    @Disabled("Waiting on radar sensor from Gulshan/Wa")
    void testRadarDetectionRange() {
        // PRE: objects at various distances
        // STEPS: test at 0.5m, 100m, 200m, 250m
        // EXPECT: detection only within 0.5-200m, 250m rejected
        fail("Not yet implemented");
    }

    // TEST CASE 026 - REQ-026
    @Test
    @Disabled("Waiting on radar sensor from Gulshan/Wa")
    void testRadarAccuracyThresholds() {
        // PRE: vary object size and weather conditions
        // STEPS: check detection threshold application
        // EXPECT: thresholds applied correctly per conditions
        fail("Not yet implemented");
    }

    // TEST CASE 027 - REQ-027
    @Test
    @Disabled("Waiting on camera sensor from Gulshan/Wa")
    void testCameraAccuracyAcrossConditions() {
        // PRE: vary light and weather conditions
        // STEPS: check classification across conditions
        // EXPECT: classification accuracy varies per conditions
        fail("Not yet implemented");
    }

    // TEST CASE 028 - REQ-028
    @Test
    @Disabled("Waiting on wheel sensor from Gulshan/Wa")
    void testWheelSpeedRange() {
        // PRE: simulate 0, 125, 250, 300 km/h
        // STEPS: read wheel speed
        // EXPECT: valid range 0-250 km/h only, 300 km/h rejected
        fail("Not yet implemented");
    }

    // TEST CASE 029 - REQ-029
    @Test
    @Disabled("Waiting on wheel sensor from Gulshan/Wa")
    void testDecelerationThreshold() {
        // PRE: rapid deceleration simulated
        // STEPS: monitor for threshold breach
        // EXPECT: braking/traction alert triggered
        fail("Not yet implemented");
    }
}