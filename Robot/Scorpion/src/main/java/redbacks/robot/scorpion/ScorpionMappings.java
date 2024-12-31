package redbacks.robot.scorpion;

import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.ButtonMapping;
import arachne4.lib.scheduler.mappings.MappingManager;
import redbacks.robot.Controllers;
import redbacks.robot.scorpion.subsystems.climber.ScorpionClimberMappings;
import redbacks.robot.scorpion.subsystems.drivetrain.ScorpionDrivetrainMappings;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerMappings;
import redbacks.robot.subsystems.noteHandling.CommonNoteHandlingMappings;
import redbacks.robot.subsystems.noteHandling.pivot.PivotMappings;

public class ScorpionMappings extends MappingManager<Controllers> {
    public ScorpionMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, Controllers.getInstance());

        new ScorpionDrivetrainMappings(schedulerProvider);
        new CommonNoteHandlingMappings(schedulerProvider);
        new PivotMappings(schedulerProvider);
        new BlowerMappings(schedulerProvider);
        new ScorpionClimberMappings(schedulerProvider);
    }

    final ButtonMapping
        targetAmp = mapping(controllers.operator::getAButton),
        targetTrap = mapping(controllers.operator::getXButton),
        targetFeedToCenter = mapping(controllers.operator::getLeftBumper),
        targetFeedToWing = mapping(controllers.operator::getRightBumper);
}
