package core.src.core.main;

import core.src.core.Interfaces.Controls.BrakingControlInterface;
import core.src.core.Interfaces.Controls.IDriverInterface;
import core.src.core.Implementions.WheelSpeedReading;

import java.util.Arrays;

public class BrakingActuatorModule {

    private final BrakingControlInterface brake;
    private final IDriverInterface driver;

    private static final long INTERVAL_MS = 50;
    private static final int MAX_RETRIES = 2;
    private static final double TOLERANCE = 0.05; // ±5%

    public BrakingActuatorModule(BrakingControlInterface brake,
                                 IDriverInterface driver) {
        this.brake = brake;
        this.driver = driver;
    }

    public void executeBraking(double brakeLevel, WheelSpeedReading[] wheels) {

        double previousRPM = getAverageRPM(wheels);

        int attempts = 0;

        while (attempts <= MAX_RETRIES) {

            long start = System.currentTimeMillis();

            // 1. Apply brake
            brake.applyBrake(brakeLevel);

            // 2. Wait until 50ms cycle completes
            waitRemaining(start);

            // 3. Read updated wheel speed
            double currentRPM = getAverageRPM(wheels);

            // 4. Verify actuation
            if (verifyDeceleration(previousRPM, currentRPM, brakeLevel)) {
                driver.showStatus("Braking successful");
                return;
            }

            attempts++;
            previousRPM = currentRPM;
        }

        // 5. Escalation
        driver.showWarning("Brake system failure!");
        driver.playSound("CRITICAL_ALERT");
    }

    // --- HELPERS ---

    private double getAverageRPM(WheelSpeedReading[] wheels) {
        return Arrays.stream(wheels)
                .mapToDouble(WheelSpeedReading::rpm)
                .average()
                .orElse(0);
    }

    private boolean verifyDeceleration(double prevRPM,
                                       double currentRPM,
                                       double brakeLevel) {

        if (prevRPM <= 0) return false;

        // expected drop based on brake level
        double expectedDrop = prevRPM * (0.1 * brakeLevel); // simple model
        double actualDrop = prevRPM - currentRPM;

        double lowerBound = expectedDrop * (1 - TOLERANCE);
        double upperBound = expectedDrop * (1 + TOLERANCE);

        return actualDrop >= lowerBound && actualDrop <= upperBound;
    }

    private void waitRemaining(long startTime) {
        long elapsed = System.currentTimeMillis() - startTime;

        if (elapsed < INTERVAL_MS) {
            try {
                Thread.sleep(INTERVAL_MS - elapsed);
            } catch (InterruptedException ignored) {}
        }
    }
}