package redbacks.lib.subsystems.positionbased;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AngleBasedIO extends PositionBasedIO<AngleBasedInputsAutoLogged, Rotation2d> {
    public static class Empty extends PositionBasedIO.Empty<AngleBasedInputsAutoLogged, Rotation2d> implements AngleBasedIO {
        public Empty() {
            super(new AngleBasedInputsAutoLogged());
        }
    }
}
