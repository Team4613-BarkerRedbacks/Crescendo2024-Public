package redbacks.robot.subsystems.noteHandling.pivot;

import arachne4.lib.Constants;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.AxisMapping;
import arachne4.lib.scheduler.mappings.ButtonMapping;
import arachne4.lib.scheduler.mappings.MappingManager;
import redbacks.robot.Controllers;

public class PivotMappings extends MappingManager<Controllers> {
    private static final PivotConstants CONSTANTS = Constants.get(PivotConstants.class);

    public static record ManualInputConstants(double joystickDeadzoneRadius, double maxPower) {}

    public final AxisMapping manualMove;

    public PivotMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, Controllers.getInstance());

        ManualInputConstants manualInputConstants = CONSTANTS.manualInputConstants();

        manualMove = mapping(() -> {
            if (!controllers.operator.getBackButton()) return 0;
            return Controllers.applyJoystickDeadzone(-controllers.operator.getLeftY(), manualInputConstants.joystickDeadzoneRadius) * manualInputConstants.maxPower;
        });
    }

    public final ButtonMapping
        autoAim = mapping(() -> Controllers.isTriggerPressed(controllers.operator.getRightTriggerAxis())),
        fixedAim = mapping(controllers.operator::getBButton),
        aimToClimb = mapping(() -> controllers.operator.getYButton() || controllers.operator.getStartButton());
}
