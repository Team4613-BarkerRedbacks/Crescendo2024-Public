package redbacks.robot.subsystems.noteHandling;

import org.littletonrobotics.junction.AutoLog;

import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.io.InputsProvider;

public abstract class NoteHandlingIO extends InputsProvider<NoteHandlingInputsAutoLogged> {
    @AutoLog
    public static class NoteHandlingInputs {
        public Measure<Velocity<Distance>>
            leftShooterVelocity = Units.MetersPerSecond.zero(),
            rightShooterVelocity = Units.MetersPerSecond.zero();

        public boolean
            isNoteInIntake = false,
            isNoteInStow = false;
    }

    public NoteHandlingIO(Scheduler scheduler) {
        super(scheduler, NoteHandlingConstants.INPUTS_LOGGING_KEY, new NoteHandlingInputsAutoLogged());
    }

    public abstract void setLeftShooterTargetVelocity(Measure<Velocity<Distance>> speed);
    public abstract void setRightShooterTargetVelocity(Measure<Velocity<Distance>> speed);

    public abstract void setIntakePercentageOutput(double power);

    public static class Empty extends NoteHandlingIO {
        public Empty(Scheduler scheduler) {
            super(scheduler);
        }

        @Override public void updateInputs() {}
        @Override public void setLeftShooterTargetVelocity(Measure<Velocity<Distance>> speed) {}
        @Override public void setRightShooterTargetVelocity(Measure<Velocity<Distance>> speed) {}
        @Override public void setIntakePercentageOutput(double power) {}
    }
}
