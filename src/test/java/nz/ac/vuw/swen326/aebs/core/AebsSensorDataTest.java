package nz.ac.vuw.swen326.aebs.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for sensor data collection and formatting.
 * REQ-009, REQ-010, REQ-011, REQ-012, REQ-013, REQ-014, REQ-015, REQ-016
 */
class AebsSensorDataTest {

    // TEST CASE 009 - REQ-009
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testRadarDataCollection() {
        // PRE: object present
        // STEPS: query radar data
        // EXPECT: distance and relative speed returned
        fail("Not yet implemented");
    }

    // TEST CASE 010 - REQ-010
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testRadarFormatting() {
        // PRE: radar has target
        // STEPS: read radar data
        // EXPECT: distance in metres, speed in km/h
        fail("Not yet implemented");
    }

    // TEST CASE 011 - REQ-011
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testRadarUpdateRate() {
        // PRE: system running
        // STEPS: monitor radar updates for 1 second
        // EXPECT: at least 10 updates (every 100ms)
        fail("Not yet implemented");
    }

    // TEST CASE 012 - REQ-012
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testCameraClassification() {
        // PRE: object present
        // STEPS: query camera sensors
        // EXPECT: object type identified
        fail("Not yet implemented");
    }

    // TEST CASE 013 - REQ-013
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testCameraUpdateRate() {
        // PRE: system running
        // STEPS: monitor camera updates for 1 second
        // EXPECT: at least 20 updates (every 50ms)
        fail("Not yet implemented");
    }

    // TEST CASE 014 - REQ-014
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testWheelRotationalSpeed() {
        // PRE: vehicle moving
        // STEPS: query wheel sensors
        // EXPECT: RPM returned per wheel
        fail("Not yet implemented");
    }

    // TEST CASE 015 - REQ-015
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testWheelDataFormat() {
        // PRE: vehicle moving
        // STEPS: read wheel data
        // EXPECT: values returned in RPM
        fail("Not yet implemented");
    }

    // TEST CASE 016 - REQ-016
    @Test
    @Disabled("Waiting on sensor interfaces from Gulshan/Wa")
    void testWheelUpdateRate() {
        // PRE: system running
        // STEPS: monitor wheel sensor for 1 second
        // EXPECT: at least 100 updates (every 10ms)
        fail("Not yet implemented");
    }
}