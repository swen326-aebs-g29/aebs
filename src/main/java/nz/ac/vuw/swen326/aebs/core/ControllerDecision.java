package nz.ac.vuw.swen326.aebs.core;

public record ControllerDecision(boolean braking, double brakeLevel, String reason) {
    public static ControllerDecision idle(String reason) {
        return new ControllerDecision(false, 0.0, reason);
    }
}
