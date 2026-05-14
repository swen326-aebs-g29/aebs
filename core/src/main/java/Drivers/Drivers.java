package core.src.core.Drivers;


import core.src.core.Interfaces.Controls.IDriverInterface;

public class Drivers implements IDriverInterface {

    private boolean active = true;
    private double sensitivity= 1.0;
    @Override
    public void showWarning(String message) {
        System.out.println("[Warning] "+ message);
    }

    @Override
    public void playSound(String sound) {
        System.out.println("[Sound] "+ sound);
    }


    @Override
    public void showVisual() {
        System.out.println("[Visual] ");

    }

    @Override
    public void showStatus(String message) {
        System.out.println("[Status] "+message);

    }

    @Override
    public void setControl() {
        System.out.println("[Control]");

    }

    @Override
    public void feedback(String message) {
        System.out.println("[Feedback] "+message);

    }

    @Override
    public void setAEBSActive(boolean active) {
        this.active = active;
        System.out.println("[AEBS] "+(active ? "active" : "inactive"));
    }

    @Override
    public boolean isAEBSActive() {
        return active;
    }




    @Override
    public void setSensitivity(double level) {
        this.sensitivity = level;
    }

    @Override
    public  double getSensitivity() {
        return sensitivity;
    }

    @Override
    public void notifyMaintenance(String message) {
        System.out.println("[Maintenance] "+message);
    }

    @Override
    public void systemReady(boolean ready) {
        System.out.println((ready ? "AEBS Ready" : "AEBS Not Ready"));
    }
}
