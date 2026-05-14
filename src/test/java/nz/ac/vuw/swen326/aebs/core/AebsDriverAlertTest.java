package nz.ac.vuw.swen326.aebs.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for driver alert and interface module.
 * REQ-001, REQ-002, REQ-003, REQ-004, REQ-005, REQ-006, REQ-007, REQ-008
 */
class AebsDriverAlertTest {

    // TEST CASE 001 - REQ-001
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testHazardAudioAlert() {
        // PRE: AEBS active, hazard detected
        // STEPS: trigger hazard proximity
        // EXPECT: auditory hazard alert plays
        fail("Not yet implemented");
    }

    // TEST CASE 002 - REQ-002
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testHazardVisualAlert() {
        // PRE: AEBS active, hazard detected
        // STEPS: trigger hazard proximity
        // EXPECT: visual hazard alert displays
        fail("Not yet implemented");
    }

    // TEST CASE 003 - REQ-003
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testSensitivityControls() {
        // PRE: system running
        // STEPS: change sensitivity/threshold settings
        // EXPECT: settings persist and affect detection
        fail("Not yet implemented");
    }

    // TEST CASE 004 - REQ-004
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testManualToggle() {
        // PRE: system running
        // STEPS: press AEBS button
        // EXPECT: AEBS turns on/off
        fail("Not yet implemented");
    }

    // TEST CASE 005 - REQ-005
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testBrakeVisualAlert() {
        // PRE: AEBS active, braking imminent
        // STEPS: trigger AEBS braking
        // EXPECT: visual brake alert displays
        fail("Not yet implemented");
    }

    // TEST CASE 006 - REQ-006
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testBrakeAuditoryAlert() {
        // PRE: AEBS active, braking imminent
        // STEPS: trigger AEBS braking
        // EXPECT: auditory brake alert plays
        fail("Not yet implemented");
    }

    // TEST CASE 007 - REQ-007
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testReadinessFeedback() {
        // PRE: system starting
        // STEPS: system initialisation
        // EXPECT: driver notified system ready
        fail("Not yet implemented");
    }

    // TEST CASE 008 - REQ-008
    @Test
    @Disabled("Waiting on DriverAlert from Gulshan")
    void testMaintenanceFeedback() {
        // PRE: system detects issue
        // STEPS: trigger detectable fault condition
        // EXPECT: maintenance alert shown
        fail("Not yet implemented");
    }
}