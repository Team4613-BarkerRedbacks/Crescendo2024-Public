package redbacks.lib.subsystems.positionbased;

import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import redbacks.lib.subsystems.positionbased.PositionVisualiser.PositionVisualiserCreator;

public class DistanceBasedSubsystem<IOT extends DistanceBasedIO> extends PositionBasedSubsystem<IOT, DistanceBasedInputsAutoLogged, Measure<Distance>> {
    public DistanceBasedSubsystem(String name, SchedulerProvider schedulerProvider,
            IOT io, Measure<Distance> startPosition,
            PositionVisualiserCreator<Measure<Distance>> visualiserCreator) {
        super(name, schedulerProvider, io, startPosition, visualiserCreator);
    }
}
