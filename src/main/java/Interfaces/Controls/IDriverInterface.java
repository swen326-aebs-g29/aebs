package Interfaces.Controls;

public interface IDriverInterface {
    void showStatus(String statusMessage);

    void feedback(String feedbackMessage);

    void showWarning(String warningMessage);

    void playSound(String soundId);

    void setControl();
}
