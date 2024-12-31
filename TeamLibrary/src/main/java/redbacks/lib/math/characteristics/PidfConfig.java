package redbacks.lib.math.characteristics;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public record PidfConfig(double kP, double kI, double kD, double kF) implements ControllerFactory<PIDFController> {
    public PIDFController createController(Measure<Time> loopPeriod) {
        return new PIDFController(kP, kI, kD, kF, loopPeriod.in(Units.Seconds));
    }
}

