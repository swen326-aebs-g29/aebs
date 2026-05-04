package core.src.core.Interfaces.Controls;

public interface IDriverInterface{
    void showWarning(String message);
    void playSound(String sound);
    void showVisual();
    void showStatus(String message);
    void setControl();
    void feedback(String message);

    void setAEBSActive(boolean active);
    boolean isAEBSActive();

    void setSensitivity(double level);
    double getSensitivity();

    void notifyMaintenance(String message);
    void systemReady(boolean ready);

    //boolean isAEBSActive();
}

