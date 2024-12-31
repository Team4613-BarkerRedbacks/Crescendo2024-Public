package redbacks.robot.io;

import redbacks.lib.subsystems.positionbased.AngleBasedIO;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;
import redbacks.robot.subsystems.noteHandling.NoteHandlingIO;

public interface IOProvider {
    DrivetrainIO getDrivetrainIO();
    NoteHandlingIO getNoteHandlingIO();
    AngleBasedIO getPivotIO();
}
