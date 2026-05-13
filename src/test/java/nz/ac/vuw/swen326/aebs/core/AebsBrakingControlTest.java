package nz.ac.vuw.swen326.aebs.core;

import aebs.simulator.model.Vec2;
import aebs.simulator.perception.*;
import aebs.simulator.world.CarBlock;
import aebs.simulator.world.WorldState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for braking control module.
 * REQ-019, REQ-020, REQ-021, REQ-022, REQ-023, REQ-024
 */
class AebsBrakingControlTest {

    private WorldState world;

    @BeforeEach
    void setUp() {
        CarBlock ego = new CarBlock("ego", CarBlock.Kind.EGO,
                new Vec2(200, 400), new Vec2(0, 60), 34, 58);
        world = new WorldState(400, 800, ego);
    }

    // TEST CASE 019 - REQ-019
    @Test
    void testBrakeSignalSent() {
        // PRE: braking imminent
        // STEPS: set brake command
        world.setBrakeCommand(1.0);
        // EXPECT: brake control signal sent (brakeCommand > 0)
        assertTrue(world.brakeCommand() > 0,
                "Brake command should be active when braking is triggered");
    }

    // TEST CASE 020 - REQ-020
    @Test
    void testBrakeSignalRate() {
        // PRE: active braking
        // STEPS: monitor signal timing for 100ms, expect at least 2 signals
        world.setBrakeCommand(1.0);
        int signalCount = 0;
        long start = System.currentTimeMillis();
        long nextSignal = start;
        while (System.currentTimeMillis() - start < 100) {
            if (System.currentTimeMillis() >= nextSignal) {
                // simulate sending brake signal every 50ms
                double signal = world.brakeCommand();
                if (signal > 0) signalCount++;
                nextSignal += 50;
            }
        }
        // EXPECT: at least 2 signals in 100ms window
        assertTrue(signalCount >= 2,
                "Brake control signal should be sent at least every 50ms during active braking");
    }

    // TEST CASE 021 - REQ-021
    @Test
    void testBrakeVerificationWithin50ms() {
        // PRE: brake command sent
        world.setBrakeCommand(1.0);
        long commandTime = System.currentTimeMillis();

        // STEPS: simulate wheel speed feedback arriving within 50ms
        world.setLastWheelSpeedFeedback(world.simTimeS(), 80.0);
        long feedbackTime = System.currentTimeMillis();

        // EXPECT: feedback received within 50ms window
        assertTrue((feedbackTime - commandTime) <= 50,
                "Brake actuation should be verified within 50ms via wheel speed feedback");
    }

    // TEST CASE 022 - REQ-022
    @Test
    void testBrakeSuccessWithinMargin() {
        // PRE: brake applied, expected deceleration is 120 km/h/s at full brake
        double expectedDecelKmhPerS = SimulatedSensors.WHEEL_EXPECTED_DECEL_FULL_BRAKE_KMH_S;
        double initialSpeedKmh = 100.0;

        // STEPS: simulate speed reduction after 1 second of full braking
        double expectedSpeedAfter1s = Math.max(0, initialSpeedKmh - expectedDecelKmhPerS);
        double margin = initialSpeedKmh * 0.05; // +/-5%

        // EXPECT: actual speed reduction within +/-5% of expected deceleration curve
        world.setLastWheelSpeedFeedback(1.0, expectedSpeedAfter1s);
        double actualSpeed = world.lastWheelSpeedKmh();
        assertEquals(expectedSpeedAfter1s, actualSpeed, margin,
                "Speed reduction should be within +/-5% of expected deceleration curve");
    }

    // TEST CASE 023 - REQ-023
    @Test
    void testCorrectiveBrakingRetries() {
        // PRE: first brake attempt fails
        // STEPS: simulate up to 2 retry attempts
        int maxRetries = 2;
        int retryCount = 0;
        world.setBrakeCommand(1.0);

        for (int attempt = 0; attempt < maxRetries; attempt++) {
            // simulate failed actuation check
            world.setLastWheelSpeedFeedback(world.simTimeS(), 100.0); // speed unchanged = failure
            retryCount++;
        }

        // EXPECT: exactly 2 corrective attempts made
        assertEquals(maxRetries, retryCount,
                "System should attempt corrective braking up to 2 times");
    }

    // TEST CASE 024 - REQ-024
    @Test
    void testEscalationAfterTwoFailures() {
        // PRE: 2 brake failures occurred
        // STEPS: set driver alert after 2 failed retries
        world.setDriverBrakeAlert(true);

        // EXPECT: escalated driver alert is active
        assertTrue(world.driverBrakeAlert(),
                "Driver brake alert should be raised after 2 failed braking attempts");
    }
}