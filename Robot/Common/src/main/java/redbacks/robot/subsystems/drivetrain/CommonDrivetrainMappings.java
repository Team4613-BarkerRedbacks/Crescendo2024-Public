package redbacks.robot.subsystems.drivetrain;

import redbacks.robot.Controllers;
import arachne4.lib.Constants;
import arachne4.lib.math.ArachneMath;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.mappings.ButtonMapping;
import arachne4.lib.scheduler.mappings.ComplexMapping;
import arachne4.lib.scheduler.mappings.MappingManager;

public class CommonDrivetrainMappings extends MappingManager<Controllers> {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

	public static final record MotionInputs2d(double vxPercent, double vyPercent, double vThetaPercent) {}

	public final ComplexMapping<MotionInputs2d> driverInputs;

    public CommonDrivetrainMappings(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, Controllers.getInstance());

		driverInputs = mapping(() -> {
			double forward = -controllers.driver.getRightY();
			double left = -controllers.driver.getRightX();
	
			double angle = Math.atan2(left, forward);
	
			// Apply linear deadzone
			double linearMagnitude = Math.sqrt(forward * forward + left * left);
			if(linearMagnitude != 0) {
				double correctedLinearMagnitude = ArachneMath.signedPow(Controllers.applyJoystickDeadzone(Math.min(1, linearMagnitude), CONSTANTS.getJoystickDeadzoneRadius()), CONSTANTS.getJoystickExponentForLinear());
				forward = correctedLinearMagnitude * Math.cos(angle);
				left = correctedLinearMagnitude * Math.sin(angle);
			}
	
			// Apply rotational deadzone
			double rotate = -Controllers.applyJoystickDeadzone(controllers.driver.getLeftX(), CONSTANTS.getJoystickDeadzoneRadius());
	
			return new MotionInputs2d(
				forward,
				left,
				ArachneMath.signedPow(rotate, CONSTANTS.getJoystickExponentForRotation())
			);
		});
    }

	protected final boolean shouldSnapToTarget() {
		return Controllers.isTriggerPressed(controllers.driver.getLeftTriggerAxis());
	}

	public final ButtonMapping 
		resetHeading = mapping(() -> controllers.driver.getStartButton() || controllers.operator.getRightStickButton())
			.readEvenWhenDisabled(),

		activateSnapTo0 = mapping(controllers.driver::getBButton),

		aimAtSpeaker = mapping(() -> shouldSnapToTarget()
			&& !controllers.operator.getAButton()
			&& !controllers.operator.getLeftBumper()
			&& !controllers.operator.getRightBumper()
			&& !controllers.operator.getXButton()),
		activateSnapToAmp = mapping(() -> shouldSnapToTarget() && controllers.operator.getAButton()),
		activateSnapToFeedCentre = mapping(() -> shouldSnapToTarget() && controllers.operator.getLeftBumper()),
		activateSnapToFeedAmp = mapping(() -> shouldSnapToTarget() && controllers.operator.getRightBumper());
}
