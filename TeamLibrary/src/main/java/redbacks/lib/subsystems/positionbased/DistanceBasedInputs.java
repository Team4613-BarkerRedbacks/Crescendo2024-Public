package redbacks.lib.subsystems.positionbased;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

@AutoLog
public class DistanceBasedInputs implements PositionBasedInputs<Measure<Distance>> {
    public Measure<Distance> currentPosition = Units.Meters.zero();
    public Measure<Distance> targetPosition = null;

    @Override
    public Measure<Distance> getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public Measure<Distance> getTargetPosition() {
        return targetPosition;
    }

    @Override
    public double convertToReadableUnits(Measure<Distance> position) {
        return position.in(Units.Meters);
    }

    @Override
    public double convertToBaseUnits(Measure<Distance> position) {
        return position.baseUnitMagnitude();
    }
}
