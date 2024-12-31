package redbacks.robot.subsystems.noteHandling;

import java.util.function.BooleanSupplier;

import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.ButtonMapping;
import arachne4.lib.scheduler.mappings.DPadDirection;
import arachne4.lib.scheduler.mappings.MappingManager;
import edu.wpi.first.units.Units;
import redbacks.lib.io.operatorinterface.AutoRumbleOnTrue;
import redbacks.robot.Controllers;

public class CommonNoteHandlingMappings extends MappingManager<Controllers>{
    public CommonNoteHandlingMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, Controllers.getInstance());
    }

    public final ButtonMapping
        outtake = mapping(() -> Controllers.isTriggerPressed(controllers.operator.getLeftTriggerAxis())),
        shoot = mapping(() -> Controllers.isTriggerPressed(controllers.driver.getRightTriggerAxis())),
        accelerateShooter = mapping(() -> Controllers.isTriggerPressed(controllers.driver.getLeftTriggerAxis())),
        shooterIntake = mapping(() -> DPadDirection.fromAngle(controllers.operator.getPOV()) == DPadDirection.DOWN);

    public final void registerRumble(BooleanSupplier sensor) {
        AutoRumbleOnTrue.register(scheduler, sensor, 1, Units.Seconds.of(0.5), controllers.driver, controllers.operator);
    }
}
