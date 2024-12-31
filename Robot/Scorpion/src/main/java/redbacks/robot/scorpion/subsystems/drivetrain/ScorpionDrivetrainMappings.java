package redbacks.robot.scorpion.subsystems.drivetrain;

import redbacks.robot.subsystems.drivetrain.CommonDrivetrainMappings;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.ButtonMapping;

public class ScorpionDrivetrainMappings extends CommonDrivetrainMappings {
    public ScorpionDrivetrainMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider);
    }

	public final ButtonMapping activateSnapToTrap = mapping(() -> shouldSnapToTarget() && controllers.operator.getXButton());
}
