package redbacks.lib.hardware.motor.ctre;

import java.util.Collection;
import java.util.List;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import redbacks.lib.math.characteristics.PidfConfig;

public class TalonFxMotorGroupIndividualControl implements TalonFxMotorGroup {
    private final List<MotorWithConfig> motorsWithConfig;

    public TalonFxMotorGroupIndividualControl(Collection<TalonFX> motors) {
        this.motorsWithConfig = motors.stream()
            .map(MotorWithConfig::new)
            .toList();
    }

    public void setPercentageOutput(DutyCycleOut output) {
        for (var motorWithConfig : motorsWithConfig) motorWithConfig.motor.setControl(output);
    }

    public void setTargetVelocityRps(VelocityDutyCycle output) {
        for (var motorWithConfig : motorsWithConfig) motorWithConfig.motor.setControl(output);
    }

    public void setTargetRotation(MotionMagicDutyCycle output) {
        for (var motorWithConfig : motorsWithConfig) motorWithConfig.motor.setControl(output);
    }

    public void setTargetRotation(PositionDutyCycle output) {
        for (var motorWithConfig : motorsWithConfig) motorWithConfig.motor.setControl(output);
    }

    public double getSelectedSensorPosition() {
        return motorsWithConfig.get(0).motor.getRotorPosition().getValue();
    }

    public void setSelectedSensorPosition(double position) {
        for (var motorWithConfig : motorsWithConfig) {
            motorWithConfig.motor.setPosition(position);
        }
    }

    public double getSelectedSensorVelocity() {
        return motorsWithConfig.get(0).motor.getRotorVelocity().getValue();
    }

    // We only set the neutral mode of the primary motor - the rest should always be in COAST mode
    public void setNeutralMode(NeutralModeValue neutralMode) {
        var config = new MotorOutputConfigs();

        motorsWithConfig.get(0).motor.getConfigurator().refresh(config);
        config.NeutralMode = neutralMode;

        motorsWithConfig.get(0).motor.getConfigurator().apply(config);
    }

    public void configPid(PidfConfig config) {
        var pidConfig = new Slot0Configs() {{
            kP = config.kP();
            kI = config.kI();
            kD = config.kD();
            kV = config.kF();
        }};

        for (var motorWithConfig : motorsWithConfig) motorWithConfig.motor.getConfigurator().apply(pidConfig);
    }

    private static class MotorWithConfig {
        private final TalonFX motor;
        private final FeedbackConfigs feedbackConfigs;

        private MotorWithConfig(TalonFX motor) {
            this.motor = motor;

            feedbackConfigs = new FeedbackConfigs();
            motor.getConfigurator().refresh(feedbackConfigs);
        }
    }
}
