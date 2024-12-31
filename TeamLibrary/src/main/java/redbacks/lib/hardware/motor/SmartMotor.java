package redbacks.lib.hardware.motor;

public interface SmartMotor {
    void setPercentageOutput(double power);
    void setToCoast();
    void setToBrake();
    void stop();
}
