package redbacks.robot.scorpion.io;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import redbacks.lib.subsystems.positionbased.AngleBasedIO;
import redbacks.lib.subsystems.positionbased.DistanceBasedIO;
import redbacks.robot.io.IOProvider;
import redbacks.robot.scorpion.subsystems.drivetrain.CameraIO;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerIO;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;
import redbacks.robot.subsystems.noteHandling.NoteHandlingIO;

public interface ScorpionIOProvider extends IOProvider {
    CameraIO getCameraIO();
    BlowerIO getBlowerIO();
    DistanceBasedIO getClimberIO();

    public static class Empty extends SchedulerProviderBase implements ScorpionIOProvider {
        public Empty(Scheduler scheduler) {
            super(scheduler);
        }

        @Override
        public DrivetrainIO getDrivetrainIO() {
            return new DrivetrainIO.Empty(getScheduler());
        }

        @Override
        public CameraIO getCameraIO() {
            return new CameraIO.Empty(getScheduler());
        }

        @Override
        public BlowerIO getBlowerIO() {
            return new BlowerIO.Empty();
        }

        @Override
        public AngleBasedIO getPivotIO() {
            return new AngleBasedIO.Empty();
        }

        @Override
        public NoteHandlingIO getNoteHandlingIO() {
            return new NoteHandlingIO.Empty(getScheduler());
        }

        @Override
        public DistanceBasedIO getClimberIO() {
            return new DistanceBasedIO.Empty();
        }
    }
}
