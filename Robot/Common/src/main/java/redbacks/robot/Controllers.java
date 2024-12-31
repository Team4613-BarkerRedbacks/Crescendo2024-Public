package redbacks.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controllers {
	private Controllers() {}

	private static final Controllers INSTANCE = new Controllers();
	public static Controllers getInstance() {
		return INSTANCE;
	}

	private static final double TRIGGER_DISTANCE_FOR_PRESS = 0.1;

	public final XboxController 
		driver = new XboxController(0),
		operator = new XboxController(1);

	public static final boolean isTriggerPressed(double triggerValue) {
		return triggerValue >= TRIGGER_DISTANCE_FOR_PRESS;
	}

    public static final double applyJoystickDeadzone(double value, double deadzoneRadius) {
        if(Math.abs(value) < deadzoneRadius) return 0;

        if(value > 0) return (value - deadzoneRadius) / (1 - deadzoneRadius);
        else return (value + deadzoneRadius) / (1 - deadzoneRadius);
    }
}
