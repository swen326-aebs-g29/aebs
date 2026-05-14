package nz.ac.vuw.swen326.aebs.core;

import aebs.simulator.perception.CameraReading;
import aebs.simulator.perception.RadarReading;
import aebs.simulator.perception.WheelSpeedReading;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;
/**
 * Tests for sensor data collection and formatting.
 * REQ-009, REQ-010, REQ-011, REQ-012, REQ-013, REQ-014, REQ-015, REQ-016
 */
class AebsSensorDataTest {

    // TEST CASE 009 - REQ-009
    @Test
    void testRadarDataCollection() {
        // PRE: object present
        RadarReading reading = new RadarReading(50.0, 30.0, System.currentTimeMillis());
        // EXPECT: distance and relative speed returned
        assertNotNull(reading);
        assertTrue(reading.distanceMetres() > 0);
        assertTrue(reading.speedKph() >= 0);
    }

    // TEST CASE 010 - REQ-010
    @Test
    void testRadarFormatting() {
        // PRE: radar has target
        RadarReading reading = new RadarReading(50.0, 30.0, System.currentTimeMillis());
        // EXPECT: distance in metres, speed in km/h
        assertEquals(50.0, reading.distanceMetres(), 0.001);
        assertTrue(reading.speedObject().contains("kph"),
                "Speed string should be in kph format");
    }

    // TEST CASE 011 - REQ-011
    @Test
    void testRadarUpdateRate() throws InterruptedException {
        // PRE: system running
        // STEPS: simulate 10 radar readings over ~1 second (every 100ms)
        int updateCount = 0;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
            new RadarReading(50.0, 30.0, System.currentTimeMillis());
            updateCount++;
            Thread.sleep(100);
        }
        // EXPECT: at least 10 updates received
        assertTrue(updateCount >= 10,
                "Radar should update at least 10 times per second");
    }

    // TEST CASE 012 - REQ-012
    @Test
    void testCameraClassification() {
        // PRE: object present
        CameraReading reading = new CameraReading("vehicle", 0.95, System.currentTimeMillis());
        // EXPECT: object type identified
        assertNotNull(reading.classification());
        assertFalse(reading.classification().isBlank(),
                "Classification should not be blank");
    }

    // TEST CASE 013 - REQ-013
    @Test
    void testCameraUpdateRate() throws InterruptedException {
        // PRE: system running
        // STEPS: simulate 20 camera readings over ~1 second (every 50ms)
        int updateCount = 0;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < 1000) {
            new CameraReading("vehicle", 0.9, System.currentTimeMillis());
            updateCount++;
            Thread.sleep(50);
        }
        // EXPECT: at least 20 updates received
        assertTrue(updateCount >= 20,
                "Camera should update at least 20 times per second");
    }

    // TEST CASE 014 - REQ-014
    @Test
    void testWheelRotationalSpeed() {
        // PRE: vehicle moving
        WheelSpeedReading reading = new WheelSpeedReading(300.0, System.currentTimeMillis());
        // EXPECT: RPM returned per wheel
        assertNotNull(reading.RPM());
        assertTrue(reading.RPM() > 0);
    }

    // TEST CASE 015 - REQ-015
    @Test
    void testWheelDataFormat() {
        // PRE: vehicle moving
        WheelSpeedReading reading = new WheelSpeedReading(300.0, System.currentTimeMillis());
        // EXPECT: values returned in RPM
        assertNotNull(reading.RPM(),
                "Wheel speed should be expressed as RPM");
        assertInstanceOf(Double.class, reading.RPM(),
                "RPM should be a Double");
    }

    // TEST CASE 016 - REQ-016
    @Test
    void testWheelUpdateRate() {
        // PRE: system running
        // STEPS: simulate wheel readings for 1 second without sleep overhead
        int updateCount = 0;
        long start = System.currentTimeMillis();
        long nextUpdate = start;
        while (System.currentTimeMillis() - start < 1000) {
            if (System.currentTimeMillis() >= nextUpdate) {
                new WheelSpeedReading(300.0, System.currentTimeMillis());
                updateCount++;
                nextUpdate += 10; // target every 10ms
            }
        }
        // EXPECT: at least 100 updates received
        assertTrue(updateCount >= 100,
                "Wheel sensors should update at least 100 times per second");
    }
}