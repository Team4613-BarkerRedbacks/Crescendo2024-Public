package redbacks.lib.math.characteristics;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;

public interface ControllerFactory<ControllerT> {
    ControllerT createController(Measure<Time> loopPeriod);
}
