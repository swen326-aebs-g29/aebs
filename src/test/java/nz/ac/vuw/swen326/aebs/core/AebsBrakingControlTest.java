package nz.ac.vuw.swen326.aebs.core;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for braking control module.
 * REQ-019, REQ-020, REQ-021, REQ-022, REQ-023, REQ-024
 */
class AebsBrakingControlTest {

    // TEST CASE 019 - REQ-019
    @Test
    @Disabled("Waiting on BrakingController from Gulshan")
    void testBrakeSignalSent() {
        // PRE: braking imminent
        // STEPS: AEBS triggers
        // EXPECT: brake control signal sent
        fail("Not yet implemented");
    }

    // TEST CASE 020 - REQ-020
    @Test
    @Disabled("Waiting on BrakingController from Gulshan")
    void testBrakeSignalRate() {
        // PRE: active braking
        // STEPS: monitor signal timing for 100ms
        // EXPECT: at least 2 signals sent (>=every 50ms)
        fail("Not yet implemented");
    }

    // TEST CASE 021 - REQ-021
    @Test
    @Disabled("Waiting on BrakingController from Gulshan")
    void testBrakeVerificationWithin50ms() {
        // PRE: brake command sent
        // STEPS: wait 50ms, check feedback
        // EXPECT: brake activation confirmed
        fail("Not yet implemented");
    }

    // TEST CASE 022 - REQ-022
    @Test
    @Disabled("Waiting on BrakingController from Gulshan")
    void testBrakeSuccessWithinMargin() {
        // PRE: brake applied
        // STEPS: compare speed reduction to expected deceleration curve
        // EXPECT: within +/-5% of expected curve
        fail("Not yet implemented");
    }

    // TEST CASE 023 - REQ-023
    @Test
    @Disabled("Waiting on BrakingController from Gulshan")
    void testCorrectiveBrakingRetries() {
        // PRE: first brake attempt fails
        // STEPS: monitor retry attempts
        // EXPECT: up to 2 retries attempted
        fail("Not yet implemented");
    }

    // TEST CASE 024 - REQ-024
    @Test
    @Disabled("Waiting on BrakingController from Gulshan")
    void testEscalationAfterTwoFailures() {
        // PRE: 2 brake failures occur
        // STEPS: trigger two consecutive failures
        // EXPECT: escalated driver alert raised
        fail("Not yet implemented");
    }
}