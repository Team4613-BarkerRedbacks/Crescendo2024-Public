package redbacks.robot.subsystems.noteHandling;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;

public class NoteHandlingConstants {
    static final String INPUTS_LOGGING_KEY = "NoteHandling";

    public static interface IntakeConstants {
        double outtakePower();
        double power();
        double shootPower();
        double slowPower();
        double reverseAfterStowOvershootPower();
        Measure<Time> slowIntakeAfterDetectionDuration();
        Measure<Time> stowReverseAfterOvershootDuration();
    }

    public interface ShooterConstants {
        GearRatio gearRatio();
        TalonFXConfiguration motorConfig();
        TalonFXConfiguration invertedMotorConfig();
        Measure<Distance> wheelDiameter();
        PidfConfig pidConfig();
        Measure<Velocity<Distance>> intakeSpeed();
        Measure<Velocity<Distance>> sweepSpeed();
        Measure<Velocity<Distance>> stowSpeed();
        Measure<Velocity<Distance>> shootSpeed();
        double generalRightRollerSpeedMultiplier();
        InterpolatingTreeMap<Double, Measure<Velocity<Distance>>> treeSpeed();
    }
}
