package redbacks.robot.scorpion.subsystems.noteHandling;

import arachne4.lib.Constants.RobotConstants;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.subsystems.noteHandling.NoteHandlingConstants.IntakeConstants;

@RobotConstants(Scorpion.class)
public class ScorpionIntakeConstants implements IntakeConstants {
    static final double
        POWER = -0.8,
        SHOOT_POWER = -0.9,
        AMP_POWER = -1,
        FEED_POWER = -1,
        OUTTAKE_POWER = 0.5,
        SLOW_POWER = -0.2,
        REVERSE_AFTER_STOW_OVERSHOOT_POWER = 0.1;

    static final Measure<Time> STOW_REVERSE_AFTER_OVERSHOOT_DURATION = Units.Seconds.of(0.3);
    static final Measure<Time> SLOW_INTAKE_AFTER_DETECTION_DURATION = Units.Seconds.of(1);

    @Override
    public double outtakePower() {
        return OUTTAKE_POWER;
    }

    @Override
    public double power() {
        return POWER;
    }

    @Override
    public double shootPower() {
        return SHOOT_POWER;
    }

    @Override
    public double slowPower() {
        return SLOW_POWER;
    }

    @Override
    public double reverseAfterStowOvershootPower() {
        return REVERSE_AFTER_STOW_OVERSHOOT_POWER;
    }

    @Override
    public Measure<Time> slowIntakeAfterDetectionDuration() {
        return SLOW_INTAKE_AFTER_DETECTION_DURATION;
    }

    @Override
    public Measure<Time> stowReverseAfterOvershootDuration() {
        return STOW_REVERSE_AFTER_OVERSHOOT_DURATION;
    }
}
