package nz.ac.vuw.swen326.aebs.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for sensor redundancy and voting logic.
 * REQ-017, REQ-018
 */
class AebsRedundancyTest {

    // TEST CASE 017 - REQ-017
    @Test
    @Disabled("Waiting on redundancy logic from Gulshan")
    void testRedundantSensorFailover() {
        // PRE: sensor A fails
        // STEPS: continue operation
        // EXPECT: sensor B provides data without interruption
        fail("Not yet implemented");
    }

    // TEST CASE 018 - REQ-018
    @Test
    @Disabled("Waiting on redundancy logic from Gulshan")
    void testMultipleSensorInputsAccepted() {
        // PRE: both sensors active
        // STEPS: read sensor data
        // EXPECT: both inputs accepted by fusion logic
        fail("Not yet implemented");
    }
}