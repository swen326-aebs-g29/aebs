package main;

import Interfaces.Controls.BrakingControlInterface;
import Interfaces.Controls.IDriverInterface;
import Implementions.WheelSpeedReading;

import java.util.Arrays;

public class BrakingControlModule {

    private final BrakingControlInterface brake;
    private final IDriverInterface driver;

    private static final long INTERVAL_MS = 50;
    private static final int MAX_RETRIES = 2;
    private static final double TOLERANCE = 0.05;

    private volatile boolean brakingActive = false;

    public BrakingControlModule(BrakingControlInterface brake,
                                IDriverInterface driver) {
        this.brake = brake;
        this.driver = driver;
    }

    // --- PUBLIC CONTROL ---

    public void startBraking(double brakeLevel, WheelSpeedReading[] wheels) {
        brakingActive = true;

        new Thread(() -> controlLoop(brakeLevel, wheels)).start();
    }

    public void stopBraking() {
        brakingActive = false;
    }

    // --- CORE LOOP ---

    private void controlLoop(double brakeLevel, WheelSpeedReading[] wheels) {

        int retries = 0;
        double previousRPM = getAverageRPM(wheels);

        while (brakingActive) {

            long cycleStart = System.nanoTime();

            // 1. Issue command (EVERY 50ms)
            brake.applyBrake(brakeLevel);

            // 2. Read current speed
            double currentRPM = getAverageRPM(wheels);

            // 3. Verify actuation within same cycle
            //boolean success = verifyDeceleration(previousRPM, currentRPM, brakeLevel);

            /*if (success) {
                retries = 0;
                driver.showStatus("Braking OK");
            } else {
                retries++;
                //REQ-023
                if (retries <= MAX_RETRIES) {
                    driver.feedback("Retry braking (" + retries + ")");
                } else {
                    //escalateFailure();
                    stopBraking();
                    return;
                }
            }*/

            previousRPM = currentRPM;

            // 4. Enforce 50ms cycle
            //sleepRemaining(cycleStart);
        }
    }
    /* REQ-21
    private void sleepRemaining(long startNano) {
        long elapsedMs = (System.nanoTime() - startNano) / 1_000_000;

        long remaining = INTERVAL_MS - elapsedMs;

        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException ignored) {}
        }
    }*/

    // --- VERIFICATION ---
    /*REQ-022
    private boolean verifyDeceleration(double prevRPM,
                                       double currentRPM,
                                       double brakeLevel) {

        if (prevRPM <= 0) return false;

        double expectedDrop = prevRPM * (0.1 * brakeLevel);
        double actualDrop = prevRPM - currentRPM;

        double lower = expectedDrop * (1 - TOLERANCE);
        double upper = expectedDrop * (1 + TOLERANCE);

        return actualDrop >= lower && actualDrop <= upper;
    }/*

    // --- ESCALATION ---

    /* REQ -024
    private void escalateFailure() {
        driver.showWarning("BRAKE FAILURE - TAKE CONTROL IMMEDIATELY");
        driver.playSound("CRITICAL_ALARM");
        driver.setControl(); // force driver takeover
    }*/

    // --- HELPERS ---

    private double getAverageRPM(WheelSpeedReading[] wheels) {
        return Arrays.stream(wheels)
                .mapToDouble(WheelSpeedReading::rpm)
                .average()
                .orElse(0);
    }

}