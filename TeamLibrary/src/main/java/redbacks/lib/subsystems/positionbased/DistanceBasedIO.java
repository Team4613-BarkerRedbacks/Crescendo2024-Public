package redbacks.lib.subsystems.positionbased;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public interface DistanceBasedIO extends PositionBasedIO<DistanceBasedInputsAutoLogged, Measure<Distance>> {
    public static class Empty extends PositionBasedIO.Empty<DistanceBasedInputsAutoLogged, Measure<Distance>> implements DistanceBasedIO {
        public Empty() {
            super(new DistanceBasedInputsAutoLogged());
        }}
}
