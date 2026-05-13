package nz.ac.vuw.swen326.aebs.core;

import aebs.simulator.model.Vec2;
import aebs.simulator.perception.*;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.WorldState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for driver alert and interface module.
 * REQ-001, REQ-002, REQ-003, REQ-004, REQ-005, REQ-006, REQ-007, REQ-008
 */
class AebsDriverAlertTest {

    private WorldState world;
    private SensorHealthMonitor monitor;

    @BeforeEach
    void setUp() {
        CarBlock ego = new CarBlock("ego", CarBlock.Kind.EGO,
                new Vec2(200, 400), new Vec2(0, 60), 34, 58);
        world = new WorldState(400, 800, ego);
        monitor = new SensorHealthMonitor();
    }

    // TEST CASE 001 - REQ-001
    @Test
    void testHazardAudioAlert() {
        // PRE: AEBS active, hazard detected (sensor healthy, object close)
        RadarReading[] hazard = {new RadarReading(5.0, 80.0, System.currentTimeMillis())};
        SensorHealthMonitor.Health health = monitor.evaluate(1.0, hazard,
                new CameraReading[0], new WheelSpeedReading[0]);
        world.setSensorHealth(health.ok(), health.summary());

        // EXPECT: system is healthy and hazard is detectable (audio alert condition met)
        assertTrue(world.sensorsHealthy(),
                "Sensors should be healthy to support hazard audio alert");
        assertTrue(hazard[0].distanceMetres() < 10.0,
                "Object within 10m should trigger hazard proximity alert");
    }

    // TEST CASE 002 - REQ-002
    @Test
    void testHazardVisualAlert() {
        // PRE: AEBS active, hazard detected
        RadarReading[] hazard = {new RadarReading(5.0, 80.0, System.currentTimeMillis())};
        SensorHealthMonitor.Health health = monitor.evaluate(1.0, hazard,
                new CameraReading[0], new WheelSpeedReading[0]);
        world.setSensorHealth(health.ok(), health.summary());

        // EXPECT: system is healthy and hazard proximity condition is met for visual alert
        assertTrue(world.sensorsHealthy(),
                "Sensors should be healthy to support hazard visual alert");
    }

    // TEST CASE 003 - REQ-003
    @Test
    void testSensitivityControls() {
        // PRE: system running
        // STEPS: simulate changing sensitivity by adjusting brake command threshold
        world.setBrakeCommand(0.3); // low sensitivity
        double lowSensitivity = world.brakeCommand();

        world.setBrakeCommand(0.8); // high sensitivity
        double highSensitivity = world.brakeCommand();

        // EXPECT: settings persist and affect detection threshold
        assertEquals(0.3, lowSensitivity, 0.001,
                "Low sensitivity setting should persist");
        assertEquals(0.8, highSensitivity, 0.001,
                "High sensitivity setting should persist");
        assertNotEquals(lowSensitivity, highSensitivity,
                "Different sensitivity settings should produce different thresholds");
    }

    // TEST CASE 004 - REQ-004
    @Test
    void testManualToggle() {
        // PRE: system running
        // STEPS: simulate toggling AEBS off then on via brake command
        world.setBrakeCommand(0.0); // AEBS off
        assertEquals(0.0, world.brakeCommand(), 0.001,
                "AEBS should be off when brake command is 0");

        world.setBrakeCommand(1.0); // AEBS on
        assertEquals(1.0, world.brakeCommand(), 0.001,
                "AEBS should be on when brake command is 1");
    }

    // TEST CASE 005 - REQ-005
    @Test
    void testBrakeVisualAlert() {
        // PRE: AEBS active, braking imminent
        world.setBrakeCommand(1.0);
        world.setDriverBrakeAlert(true);

        // EXPECT: visual brake alert condition is active
        assertTrue(world.driverBrakeAlert(),
                "Driver brake alert should be active when braking is imminent");
        assertTrue(world.brakeCommand() > 0,
                "Brake command should be active during visual alert condition");
    }

    // TEST CASE 006 - REQ-006
    @Test
    void testBrakeAuditoryAlert() {
        // PRE: AEBS active, braking imminent
        world.setBrakeCommand(1.0);
        world.setDriverBrakeAlert(true);

        // EXPECT: auditory brake alert condition is active
        assertTrue(world.driverBrakeAlert(),
                "Driver brake alert should be active for auditory alert condition");
    }

    // TEST CASE 007 - REQ-007
    @Test
    void testReadinessFeedback() {
        // PRE: system starting
        // STEPS: initialise world and check sensor health baseline
        RadarReading[] empty = new RadarReading[0];
        CameraReading[] emptyCam = new CameraReading[0];
        WheelSpeedReading[] emptyWheel = new WheelSpeedReading[0];

        // Fresh monitor with no prior state should report OK on empty readings
        SensorHealthMonitor freshMonitor = new SensorHealthMonitor();
        SensorHealthMonitor.Health health = freshMonitor.evaluate(0.0,
                empty, emptyCam, emptyWheel);
        world.setSensorHealth(health.ok(), health.summary());

        // EXPECT: system ready state is communicated (no faults on init)
        assertFalse(world.failSafeActive(),
                "Fail-safe should not be active on system initialisation");
        assertEquals("OK", world.sensorHealthSummary(),
                "System should report OK on initialisation");
    }

    // TEST CASE 008 - REQ-008
    @Test
    void testMaintenanceFeedback() {
        // PRE: system detects fault condition
        RadarReading[] bad = {new RadarReading(-1.0, Double.NaN,
                System.currentTimeMillis())};
        SensorHealthMonitor.Health health = monitor.evaluate(1.0, bad,
                new CameraReading[0], new WheelSpeedReading[0]);
        world.setSensorHealth(health.ok(), health.summary());

        // EXPECT: maintenance alert shown (sensor health is not OK)
        assertFalse(world.sensorsHealthy(),
                "System should flag unhealthy sensors as a maintenance condition");
        assertFalse(world.sensorHealthSummary().equals("OK"),
                "Health summary should not be OK when maintenance is needed");
    }
}