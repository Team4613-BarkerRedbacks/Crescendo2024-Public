package redbacks.robot.scorpion.subsystems.noteHandling;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import arachne4.lib.Constants.RobotConstants;
import arachne4.lib.math.Interpolators;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.subsystems.noteHandling.NoteHandlingConstants.ShooterConstants;

@RobotConstants(Scorpion.class)
public class ScorpionShooterConstants implements ShooterConstants {
    static final double
        GENERAL_RIGHT_ROLLER_SPEED_MULTIPLER = 0.6,
        RIGHT_ROLLER_SPEED_FOR_AMP_MULTIPLIER = 1;

    static final Measure<Velocity<Distance>> 
        MAXIMUM_AMP_SPEED = Units.MetersPerSecond.of(5.8),
        MAXIMUM_FEED_SPEED = Units.MetersPerSecond.of(30),
        THRESHOLD_FOR_FEEDING = Units.MetersPerSecond.of(3),
        MINIMUM_SPEAKER_SPEED = Units.MetersPerSecond.of(19),
        MINIMUM_OFFSET_SPEED = Units.MetersPerSecond.of(3);

    static final Measure<Velocity<Distance>> 
        SHOOT_SPEED = Units.MetersPerSecond.of(30),
        STOW_SPEED = Units.MetersPerSecond.of(5),
        AMP_SPEED = Units.MetersPerSecond.of(4.9),
        FEED_TO_CENTRE_SPEED = Units.MetersPerSecond.of(19),
        FEED_TO_AMP_SPEED = Units.MetersPerSecond.of(20),
        INTAKE_SPEED = Units.MetersPerSecond.of(-6),
        TRAP_SPEED = Units.MetersPerSecond.of(9.8);


    static final GearRatio 
        SHOOTER_GEAR_RATIO = new GearRatio(1, 1.5 / (29d/39d) * (25d/39d));

    public static final Measure<Velocity<Distance>> SWEEP_SPEED = Units.MetersPerSecond.of(3.5);

    static final Measure<Distance> 
        SHOOTER_WHEEL_DIAMETER = Units.Inches.of(4);

    static final PidfConfig PID_CONFIG = new PidfConfig(
        0.03, 
        0,
        0.0003, 
        0.75 / 70 
    );

    static final TalonFXConfiguration 
    INVERTED_MOTOR_CONFIG = new TalonFXConfiguration()
        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
        .withDutyCycleClosedLoopRampPeriod(0.5))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)),
    MOTOR_CONFIG = new TalonFXConfiguration()
        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
        .withDutyCycleClosedLoopRampPeriod(0.5));

    static final InterpolatingTreeMap<Double, Measure<Velocity<Distance>>> TREE_SPEED =
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forMeasure())
        {{
            put(1.34, Units.MetersPerSecond.of(24));
            put(2.50, Units.MetersPerSecond.of(27));
            put(4.0, Units.MetersPerSecond.of(30));
        }};

    static final InterpolatingTreeMap<Double, Measure<Velocity<Distance>>> FEED_CENTRE_TREE_SPEED =
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forMeasure())
        {{
            put(2.0, Units.MetersPerSecond.of(10));
            put(6.0, Units.MetersPerSecond.of(14));
            put(7.0, Units.MetersPerSecond.of(18));
            put(9.0, Units.MetersPerSecond.of(20));

        }};

    static final InterpolatingTreeMap<Double, Measure<Velocity<Distance>>> FEED_AMP_TREE_SPEED =
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forMeasure())
        {{
            put(2.0, Units.MetersPerSecond.of(5));
            put(6.0, Units.MetersPerSecond.of(18));
            put(7.0, Units.MetersPerSecond.of(21));
        }};

    @Override
    public GearRatio gearRatio() {
        return SHOOTER_GEAR_RATIO;
    }

    @Override
    public TalonFXConfiguration motorConfig() {
        return MOTOR_CONFIG;
    }

    @Override
    public TalonFXConfiguration invertedMotorConfig() {
        return INVERTED_MOTOR_CONFIG;
    }

    @Override
    public Measure<Distance> wheelDiameter() {
        return SHOOTER_WHEEL_DIAMETER;
    }

    @Override
    public PidfConfig pidConfig() {
        return PID_CONFIG;
    }

    @Override
    public Measure<Velocity<Distance>> intakeSpeed() {
        return INTAKE_SPEED;
    }

    @Override
    public Measure<Velocity<Distance>> sweepSpeed() {
        return SWEEP_SPEED;
    }

    @Override
    public Measure<Velocity<Distance>> stowSpeed() {
        return STOW_SPEED;
    }

    @Override
    public Measure<Velocity<Distance>> shootSpeed() {
        return SHOOT_SPEED;
    }

    @Override
    public double generalRightRollerSpeedMultiplier() {
        return GENERAL_RIGHT_ROLLER_SPEED_MULTIPLER;
    }

    @Override
    public InterpolatingTreeMap<Double, Measure<Velocity<Distance>>> treeSpeed() {
        return TREE_SPEED;
    }
} 
