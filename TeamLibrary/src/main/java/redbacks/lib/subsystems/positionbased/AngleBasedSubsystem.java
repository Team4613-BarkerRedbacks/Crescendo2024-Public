package redbacks.lib.subsystems.positionbased;

import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.math.geometry.Rotation2d;
import redbacks.lib.subsystems.positionbased.PositionVisualiser.PositionVisualiserCreator;

public abstract class AngleBasedSubsystem<IOT extends AngleBasedIO> extends PositionBasedSubsystem<IOT, AngleBasedInputsAutoLogged, Rotation2d> {
    public AngleBasedSubsystem(String name, SchedulerProvider schedulerProvider,
            IOT io, Rotation2d startPosition,
            PositionVisualiserCreator<Rotation2d> visualiserCreator) {
        super(name, schedulerProvider, io, startPosition, visualiserCreator);
    }
}
