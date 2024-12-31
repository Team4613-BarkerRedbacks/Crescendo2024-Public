package redbacks.lib.subsystems.positionbased;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

@AutoLog
public class AngleBasedInputs implements PositionBasedInputs<Rotation2d> {
    public Rotation2d currentPosition = Rotation2d.fromRadians(0);
    public Rotation2d targetPosition = null;

    @Override
    public Rotation2d getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public Rotation2d getTargetPosition() {
        return targetPosition;
    }

    @Override
    public double convertToReadableUnits(Rotation2d position) {
        return position.getDegrees();
    }

    @Override
    public double convertToBaseUnits(Rotation2d position) {
        return position.getRadians();
    }
}
