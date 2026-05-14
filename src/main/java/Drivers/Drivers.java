package core.src.core.Drivers;

import core.src.core.Interfaces.Controls.IDriverInterface;

public class Drivers implements IDriverInterface {

    private boolean active = true;
    private double sensitivity = 1.0;

    // --- ALERT TYPES ---
    public enum AlertLevel {
        INFO,
        WARNING,
        CRITICAL
    }

    // --- CORE ALERT METHODS ---

    @Override
    public void showWarning(String message) {
        triggerVisual(AlertLevel.WARNING, message);
        triggerAudio("warning_beep");
    }

    @Override
    public void playSound(String sound) {
        System.out.println("[AUDIO] " + sound);
    }

    @Override
    public void showVisual() {
        System.out.println("[VISUAL] Flashing dashboard indicator");
    }

    @Override
    public void showStatus(String message) {
        System.out.println("[STATUS] " + message);
    }

    @Override
    public void setControl() {
        System.out.println("[CONTROL] Driver override engaged");
    }

    @Override
    public void feedback(String message) {
        System.out.println("[FEEDBACK] " + message);
    }

    // --- AEBS STATE CONTROL ---

    @Override
    public void setAEBSActive(boolean active) {
        this.active = active;

        if (active) {
            triggerVisual(AlertLevel.INFO, "AEBS ON");
            triggerAudio("system_on");
        } else {
            triggerVisual(AlertLevel.INFO, "AEBS OFF");
            triggerAudio("system_off");
        }
    }

    @Override
    public boolean isAEBSActive() {
        return active;
    }

    // --- SENSITIVITY CONTROL ---

    @Override
    public void setSensitivity(double level) {
        if (level < 0.1 || level > 2.0) {
            feedback("Sensitivity out of range (0.1 - 2.0)");
            return;
        }

        this.sensitivity = level;
        feedback("Sensitivity set to " + level);
    }

    @Override
    public double getSensitivity() {
        return sensitivity;
    }

    // --- MAINTENANCE + READINESS ---

    @Override
    public void notifyMaintenance(String message) {
        triggerVisual(AlertLevel.CRITICAL, "Maintenance: " + message);
        triggerAudio("maintenance_alert");
    }

    @Override
    public void systemReady(boolean ready) {
        if (ready) {
            triggerVisual(AlertLevel.INFO, "System Ready");
            triggerAudio("ready_chime");
        } else {
            triggerVisual(AlertLevel.WARNING, "System Not Ready");
            triggerAudio("error_tone");
        }
    }

    // --- INTERNAL HELPERS ---

    private void triggerVisual(AlertLevel level, String message) {
        System.out.println("[VISUAL - " + level + "] " + message);
    }

    private void triggerAudio(String sound) {
        System.out.println("[AUDIO] " + sound);
    }
}