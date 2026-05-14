package core.src.main.java.main;

import core.src.main.java.Interfaces.Controls.BrakingControlInterface;

public class SimulatedBrakeControl implements BrakingControlInterface {
    public void applyBrake(double v) {
        System.out.println("Simulated brake: " + v);
    }

}
