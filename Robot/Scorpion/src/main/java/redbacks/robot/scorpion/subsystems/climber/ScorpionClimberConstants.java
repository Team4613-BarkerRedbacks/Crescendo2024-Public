package redbacks.robot.scorpion.subsystems.climber;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;

public class ScorpionClimberConstants {
    static final String INPUTS_LOGGING_KEY = "Climber";

    static final double CLIMBER_HIGHER_POWER = 1;
    static final double CLIMBER_LOWER_POWER = -0.5;

    static final double REVOLUTIONS_PER_METRE = 185 / 0.7;

    static final PidfConfig PID_CONFIG = new PidfConfig(
        0.03, 
        0,
        0.0003, 
        0.75 / 70 
    );

    static final Measure<Distance>
        START_POSITION = Units.Meters.of(0.21),
        UP_POSITION = Units.Meters.of(0.91);

    static final GearRatio GEAR_RATIO = new GearRatio(25, 1);

    static final TalonFXConfiguration
        MOTOR_CONFIG = new TalonFXConfiguration()
            .withClosedLoopRamps(new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.5))
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
}
