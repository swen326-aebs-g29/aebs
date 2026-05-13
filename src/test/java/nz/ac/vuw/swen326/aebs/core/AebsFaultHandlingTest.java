package nz.ac.vuw.swen326.aebs.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for fault detection, tolerance, and fail-safe behaviour.
 * REQ-030, REQ-031, REQ-032
 */
class AebsFaultHandlingTest {

    // TEST CASE 030 - REQ-030
    @Test
    @Disabled("Waiting on fault detection from Gulshan/Wa")
    void testFaultDetection() {
        // PRE: inject abnormal sensor value
        // STEPS: system running with injected fault
        // EXPECT: fault detected and reported
        fail("Not yet implemented");
    }

    // TEST CASE 031 - REQ-031
    @Test
    @Disabled("Waiting on fault tolerance from Gulshan/Wa")
    void testFaultTolerance() {
        // PRE: sensor fails during active hazard
        // STEPS: check hazard response continues
        // EXPECT: safe hazard management continues despite failure
        fail("Not yet implemented");
    }

    // TEST CASE 032 - REQ-032
    @Test
    @Disabled("Waiting on fail-safe mode from Gulshan/Wa")
    void testFailSafeActivation() {
        // PRE: normal operation impossible
        // STEPS: trigger critical system failure
        // EXPECT: fail-safe mode activated
        fail("Not yet implemented");
    }
}