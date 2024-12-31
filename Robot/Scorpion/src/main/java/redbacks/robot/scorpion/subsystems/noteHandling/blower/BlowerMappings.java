package redbacks.robot.scorpion.subsystems.noteHandling.blower;

import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.MappingManager;
import redbacks.robot.Controllers;

public class BlowerMappings extends MappingManager<Controllers> {
    public BlowerMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, Controllers.getInstance());
    }
}
