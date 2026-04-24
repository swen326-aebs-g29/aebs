package core.src.core.main;

import core.src.core.Interfaces.Controls.BrakingControlInterface;

public class SimulatedBrakeControl implements BrakingControlInterface {
    public void applyBrake(double v) {
        System.out.println("Simulated brake: " + v);
    }

}
