package redbacks.robot.scorpion.io;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import redbacks.lib.subsystems.positionbased.AngleBasedIO;
import redbacks.lib.subsystems.positionbased.DistanceBasedIO;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.scorpion.subsystems.climber.ScorpionClimberHardware;
import redbacks.robot.scorpion.subsystems.drivetrain.CameraHardware;
import redbacks.robot.scorpion.subsystems.drivetrain.CameraIO;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerHardware;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerIO;
import redbacks.robot.subsystems.drivetrain.DrivetrainHardware;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;
import redbacks.robot.subsystems.noteHandling.NoteHandlingHardware;
import redbacks.robot.subsystems.noteHandling.NoteHandlingIO;
import redbacks.robot.subsystems.noteHandling.pivot.PivotHardware;

public class ScorpionHardware extends SchedulerProviderBase implements ScorpionIOProvider {
    private final DrivetrainHardware drivetrainHardware;
    private final CameraHardware cameraHardware;
    private final NoteHandlingHardware noteHandlingHardware;
    private final PivotHardware pivotHardware;
    private final BlowerHardware blowerHardware;
    private final ScorpionClimberHardware climberHardware;

    public ScorpionHardware(Scheduler scheduler, Scorpion robot) {
        super(scheduler);

        this.drivetrainHardware = new DrivetrainHardware(this);
        this.cameraHardware = new CameraHardware(this, robot);
        this.noteHandlingHardware = new NoteHandlingHardware(this);
        this.pivotHardware = new PivotHardware(this);
        this.blowerHardware = new BlowerHardware(this);
        this.climberHardware = new ScorpionClimberHardware(this);
    }

    @Override
    public DrivetrainIO getDrivetrainIO() {
        return drivetrainHardware;
    }

    @Override
    public CameraIO getCameraIO() {
        return cameraHardware;
    }

    @Override
    public NoteHandlingIO getNoteHandlingIO() {
        return noteHandlingHardware;
    }

    @Override
    public AngleBasedIO getPivotIO() {
        return pivotHardware;
    }

    @Override
    public BlowerIO getBlowerIO() {
        return blowerHardware;
    }

    @Override
    public DistanceBasedIO getClimberIO() {
        return climberHardware;
    }
}
