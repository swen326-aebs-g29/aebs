package main;

import Interfaces.Controls.BrakingControlInterface;
import Interfaces.Controls.IDriverInterface;
import Implementions.WheelSpeedReading;

import java.util.Arrays;
import java.util.Objects;
import java.util.function.Supplier;

public class BrakingControlModule {

    private final BrakingControlInterface brake;
    private final IDriverInterface driver;


    private static final long INTERVAL_MS = 50;
    private static final int MAX_RETRIES = 2;
    private static final double TOLERANCE = 0.05;

    private volatile boolean brakingActive = false;
    private volatile double requestedBrakeLevel = 0.0;
    private volatile Supplier<WheelSpeedReading[]> wheelSupplier = () -> new WheelSpeedReading[0];


    public BrakingControlModule(BrakingControlInterface brake,
                                IDriverInterface driver) {
        this.brake = brake;
        this.driver = driver;

    }

    // --- PUBLIC CONTROL ---

    public void startBraking(double brakeLevel, WheelSpeedReading[] wheels) {
        startBraking(brakeLevel, () -> wheels);
    }

    public void startBraking(double brakeLevel, Supplier<WheelSpeedReading[]> wheelSupplier) {
        this.requestedBrakeLevel = clampBrakeLevel(brakeLevel);
        this.wheelSupplier = Objects.requireNonNull(wheelSupplier, "wheelSupplier");

        if (brakingActive) {
            return;
        }

        brakingActive = true;
        Thread worker = new Thread(this::controlLoop, "braking-control-module");
        worker.setDaemon(true);
        worker.start();
    }

    public void stopBraking() {
        brakingActive = false;
        requestedBrakeLevel = 0.0;
        brake.applyBrake(0.0);
    }

    // --- CORE LOOP ---

    private void controlLoop() {
        int retries = 0;
        double previousRPM = getAverageRPM(wheelSupplier.get());

        while (brakingActive) {
            long cycleStart = System.nanoTime();
            double brakeLevel = requestedBrakeLevel;
            WheelSpeedReading[] wheels = wheelSupplier.get();

            // 1. Issue command (EVERY 50ms)
            brake.applyBrake(brakeLevel);

            // 2. Read current speed
            double currentRPM = getAverageRPM(wheels);

            // 3. Verify actuation within same cycle
            boolean success = verifyDeceleration(previousRPM, currentRPM, brakeLevel);

            if (success) {
                retries = 0;
                driver.showStatus("Braking OK");
            } else {

                //REQ-023
                if (retries < MAX_RETRIES) {
                    retries++;
                    driver.feedback("Retry braking (" + retries + ")");
                    applyCorrectiveBrake(brakeLevel);
                } else {
                    escalateFailure();
                    stopBraking();
                    return;
                }
            }

            previousRPM = currentRPM;

            // 4. Enforce 50ms cycle
            sleepRemaining(cycleStart);
        }
    }
    //REQ-20
    private void sleepRemaining(long startNano) {
        long elapsedMs = (System.nanoTime() - startNano) / 1_000_000;

        long remaining = INTERVAL_MS - elapsedMs;

        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    // --- VERIFICATION ---
    //REQ-022
    private boolean verifyDeceleration(
            double previousRPM,
            double currentRPM,
            double brakeLevel)
    {
        // 50 ms cycle
        double deltaTime = 0.05;

        // Actual RPM reduction
        double actualDecel =
                (previousRPM - currentRPM)
                        / deltaTime;

        // Expected curve
        double expectedDecel =
                expectedDeceleration(brakeLevel);

        // Percentage error
        if(expectedDecel <= 0) { return false; }
        double error =
                Math.abs(actualDecel
                        - expectedDecel)
                        / expectedDecel;

        return error <= TOLERANCE;
    }

    // --- ESCALATION ---

    //REQ -024
    private void escalateFailure() {
        driver.showWarning("BRAKE FAILURE - TAKE CONTROL IMMEDIATELY");
        driver.playSound("CRITICAL_ALARM");
        driver.setControl(); // force driver takeover
    }

    // --- HELPERS ---

    private double getAverageRPM(WheelSpeedReading[] wheels) {
        if (wheels == null || wheels.length == 0) {
            return 0.0;
        }

        return Arrays.stream(wheels)
                .filter(Objects::nonNull)
                .mapToDouble(WheelSpeedReading::rpm)
                .average()
                .orElse(0);
    }

    private void applyCorrectiveBrake(double brakeLevel) {
        double correctedLevel = Math.min(brakeLevel + 0.1, 1.0);

        brake.applyBrake(correctedLevel);
    }

    private double expectedDeceleration(double brakeLevel) {
        return brakeLevel * 500;
    }

    private double clampBrakeLevel(double brakeLevel) {
        return Math.max(0.0, Math.min(1.0, brakeLevel));
    }

}