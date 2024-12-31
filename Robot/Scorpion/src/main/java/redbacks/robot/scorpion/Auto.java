package redbacks.robot.scorpion;

import static arachne4.lib.sequences.Actionable.*;
import arachne4.lib.sequences.Actionable;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.Function;

public enum Auto implements Function<Scorpion, Actionable> {
	DO_NOTHING((robot) -> DO_NOTHING()),	
	
	RED_7_NOTE(AutoDduCenter.createForAlliance(Alliance.Red)),
	BLUE_7_NOTE(AutoDduCenter.createForAlliance(Alliance.Blue)),

	RED_LOADING_SIDE(AutoDduLoadingSide.createForAlliance(Alliance.Red)),
	BLUE_LOADING_SIDE(AutoDduLoadingSide.createForAlliance(Alliance.Blue));

	private static double autoStartTime;

	private final Function<Scorpion, Actionable> actionableGenerator;

	private Auto(Function<Scorpion, Actionable> actionableGenerator) {
		this.actionableGenerator = actionableGenerator;
	}

	@Override
	public Actionable apply(Scorpion robot) {
		return SPLIT(
			actionableGenerator.apply(robot)
		).AND(
			DO(() -> autoStartTime = Timer.getFPGATimestamp())
		);
	}

	static boolean doesTimeSinceAutoStartExceed(Measure<Time> time) {
		return Timer.getFPGATimestamp() - autoStartTime >= time.in(Units.Seconds);
	}
}