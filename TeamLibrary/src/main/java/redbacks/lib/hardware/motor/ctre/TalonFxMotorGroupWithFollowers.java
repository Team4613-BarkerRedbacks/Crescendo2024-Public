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

public class TalonFxMotorGroupWithFollowers implements TalonFxMotorGroup {
    private final TalonFX primaryMotor;
    private final List<TalonFX> motors;
    private final FeedbackConfigs feedbackConfigs;

    public TalonFxMotorGroupWithFollowers(TalonFX primaryMotor, Collection<TalonFX> motors) {
        this.primaryMotor = primaryMotor;
        this.motors = List.copyOf(motors);

        this.feedbackConfigs = new FeedbackConfigs();
        primaryMotor.getConfigurator().refresh(feedbackConfigs);
    }

    public void setPercentageOutput(DutyCycleOut output) {
        primaryMotor.setControl(output);
    }

    public void setTargetVelocityRps(VelocityDutyCycle output) {
        primaryMotor.setControl(output);
    }

    public void setTargetRotation(MotionMagicDutyCycle output) {
        primaryMotor.setControl(output);
    }

    public void setTargetRotation(PositionDutyCycle output) {
        primaryMotor.setControl(output);
    }

    public double getSelectedSensorPosition() {
        return primaryMotor.getRotorPosition().getValue();
    }

    public void setSelectedSensorPosition(double position) {
        primaryMotor.setPosition(position);
    }

    public double getSelectedSensorVelocity() {
        return primaryMotor.getRotorVelocity().getValue();
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        for(var motor : motors) {
            var config = new MotorOutputConfigs();

            motor.getConfigurator().refresh(config);
            config.NeutralMode = neutralMode;

            motor.getConfigurator().apply(config);
        }
    }

    public void configPid(PidfConfig config) {
        var pidConfig = new Slot0Configs() {{
            kP = config.kP();
            kI = config.kI();
            kD = config.kD();
            kV = config.kF();
        }};

        primaryMotor.getConfigurator().apply(pidConfig);
    }
}
