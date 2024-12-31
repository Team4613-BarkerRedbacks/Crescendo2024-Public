package arachne4.lib;

import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import arachne4.lib.game.GameState;
import arachne4.lib.logging.ArachneLogger;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.data.ValueChange;
import arachne4.lib.subsystems.Subsystem;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class ArachneRobotWithAdvantageKit<RobotT extends Subsystem> extends LoggedRobot implements SchedulerProvider {
    private static final ArachneLogger log = ArachneLogger.getInstance();

    protected final Supplier<RobotT> robotCreator;
    protected final Class<RobotT> robotClass;
    protected final Scheduler scheduler;
    protected final Mode mode;

	protected GameState gameState = GameState.PRE_INIT;

    protected ArachneRobotWithAdvantageKit(BiFunction<SchedulerProvider, Mode, RobotT> robotSupplier, Class<RobotT> robotClass, Measure<Time> loopPeriod, Mode mode) {
        super(loopPeriod.in(Units.Seconds));

        this.robotCreator = () -> robotSupplier.apply(this, mode);
        this.robotClass = robotClass;
		this.scheduler = new Scheduler();
        this.mode = mode;
	}

	public static <RobotT extends Subsystem> void startArachneRobotWithAdvantageKit(BiFunction<SchedulerProvider, Mode, RobotT> robotSupplier, Class<RobotT> robotClass, Mode mode) {
		startArachneRobotWithAdvantageKit(robotSupplier, robotClass, ArachneRobot4.DEFAULT_PERIOD, mode);
	}

	public static <RobotT extends Subsystem> void startArachneRobotWithAdvantageKit(BiFunction<SchedulerProvider, Mode, RobotT> robotSupplier, Class<RobotT> robotClass, Measure<Time> loopPeriod, Mode mode) {
        log.info(String.format("Starting robot %s in %s mode", robotClass.getSimpleName(), mode));
        startRobot(() -> new ArachneRobotWithAdvantageKit<>(robotSupplier, robotClass, loopPeriod, mode));
	}

    public static enum Mode {
        REAL,
        SIMULATED,
        REPLAY;
    }

	@Override
	public Scheduler getScheduler() {
		return scheduler;
	}

    // ----------------------------------------
    // Initialisation
    // ----------------------------------------

    @Override
    public void robotInit() {
        // Set up data receivers & replay source
        switch (mode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
            break;

            case SIMULATED:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
            break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
            break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        Constants.setRobotClass(robotClass);
		robotCreator.get();
        fire(Scheduler.INITIALIZE);
    }

    @Override
    public void endCompetition() {
        super.endCompetition();
		fire(Scheduler.END);
    }

    // ----------------------------------------
    // Game state changes
    // ----------------------------------------

    protected void onGameStateChangeTo(GameState newGameState) {
        try {
            fire(Scheduler.GAME_STATE_CHANGE, new ValueChange<>(gameState, newGameState));
        }
        catch (Exception exception) {
            Logger.recordOutput(String.format("UncaughtError/%s-to-%s", gameState.name(), newGameState.name()), exception.toString());
		    DriverStation.reportError(exception.getMessage(), exception.getStackTrace());
        }

        gameState = newGameState;
    }

    @Override public void autonomousInit() { onGameStateChangeTo(GameState.AUTO); }
    @Override public void teleopInit() { onGameStateChangeTo(GameState.TELEOP); }
    @Override public void disabledInit() { onGameStateChangeTo(GameState.DISABLED); }
    @Override public void testInit() { onGameStateChangeTo(GameState.TEST); }

    // ----------------------------------------
    // Periodic executions
    // ----------------------------------------

    protected void onExecute(GameState gameState) {
        try {
            fire(Scheduler.PRE_EXECUTE, gameState);
            fire(Scheduler.EXECUTE, gameState);
            fire(Scheduler.POST_EXECUTE, gameState);
        }
        catch (Exception exception) {
            Logger.recordOutput(String.format("UncaughtError/%s-periodic", gameState.name()), exception.toString());
		    DriverStation.reportError(exception.getMessage(), exception.getStackTrace());
        }
    }

    @Override public void robotPeriodic() {}
    @Override public void autonomousPeriodic() { onExecute(GameState.AUTO); }
    @Override public void teleopPeriodic() { onExecute(GameState.TELEOP); }
    @Override public void disabledPeriodic() { onExecute(GameState.DISABLED); }
    @Override public void testPeriodic() { onExecute(GameState.TEST); }

    // ----------------------------------------
    // Simulation
    // ----------------------------------------

    @Override public void simulationInit() { fire(Scheduler.SIMULATION_INIT); }
    @Override public void simulationPeriodic() { fire(Scheduler.SIMULATION_EXECUTE); }
}
