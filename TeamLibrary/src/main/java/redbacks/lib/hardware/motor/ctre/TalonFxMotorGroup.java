package redbacks.lib.hardware.motor.ctre;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import redbacks.lib.math.characteristics.PidfConfig;

public interface TalonFxMotorGroup {
    void setPercentageOutput(DutyCycleOut output);
    void setTargetVelocityRps(VelocityDutyCycle output);
    void setTargetRotation(MotionMagicDutyCycle output);
    void setTargetRotation(PositionDutyCycle output);

    double getSelectedSensorPosition();
    void setSelectedSensorPosition(double position);

    double getSelectedSensorVelocity();

    void setNeutralMode(NeutralModeValue neutralMode);
    void configPid(PidfConfig config);
}
