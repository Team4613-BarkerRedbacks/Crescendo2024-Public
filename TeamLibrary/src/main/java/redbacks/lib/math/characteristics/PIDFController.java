package redbacks.lib.math.characteristics;

import edu.wpi.first.math.controller.PIDController;

public class PIDFController extends PIDController {
    private double kf;

    public PIDFController(double kp, double ki, double kd, double kf, double period) {
        super(kp, ki, kd, period);
        this.kf = kf;
    }

    public void setPIDF(double kp, double ki, double kd, double kf) {
        setPID(kp, ki, kd);
        setF(kf);
    }

    public void setF(double kf) {
        this.kf = kf;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        return super.calculate(measurement, setpoint) + getFeedForwardForSetpoint(setpoint);
    }

    @Override
    public double calculate(double measurement) {
        return super.calculate(measurement) + getFeedForwardForSetpoint(getSetpoint());
    }

    private double getFeedForwardForSetpoint(double setpoint) {
        return kf * setpoint;
    }
}
