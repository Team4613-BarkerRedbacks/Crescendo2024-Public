package arachne4.lib.game;

import edu.wpi.first.hal.DriverStationJNI;

public enum GameState {
	PRE_INIT(() -> {}), // Default non-null state the robot transitions from when initialising into DISABLED
	DISABLED(DriverStationJNI::observeUserProgramDisabled),
	AUTO(DriverStationJNI::observeUserProgramAutonomous),
	TELEOP(DriverStationJNI::observeUserProgramTeleop),
	TEST(DriverStationJNI::observeUserProgramTest);

	public static final GameState[] DRIVER_CONTROLLED_STATES = {TELEOP, TEST};

	public final Runnable halObserver;

	private GameState(Runnable halObserver) {
		this.halObserver = halObserver;
	}
}
