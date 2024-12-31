package redbacks.robot.scorpion.subsystems.climber;

import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.AxisMapping;
import arachne4.lib.scheduler.mappings.ButtonMapping;
import arachne4.lib.scheduler.mappings.MappingManager;
import redbacks.robot.Controllers;

public class ScorpionClimberMappings  extends MappingManager<Controllers> {
    public ScorpionClimberMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, Controllers.getInstance());
    }

    static final double
    MANUAL_MAX_POWER = 0.2,
    MANUAL_JOYSTICK_DEADZONE_RADIUS = 0.3;

    public final ButtonMapping climbDown = mapping(() -> controllers.operator.getStartButton() && -controllers.operator.getLeftY() < -0.3);
    public final ButtonMapping climbUp = mapping(() -> controllers.operator.getStartButton() && -controllers.operator.getLeftY() > 0.3);

    public final ButtonMapping climbUpImmediately = mapping(() -> controllers.operator.getYButton());

    final AxisMapping manualMove = mapping(() -> {
        if (!controllers.operator.getStartButton()) return 0;
        return Controllers.applyJoystickDeadzone(-controllers.operator.getLeftY(), MANUAL_JOYSTICK_DEADZONE_RADIUS) * MANUAL_MAX_POWER;
    });
}