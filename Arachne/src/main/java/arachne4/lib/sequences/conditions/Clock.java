package arachne4.lib.sequences.conditions;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public class Clock
{
	private Clock() {}
	
	public static BooleanSupplier delay(Measure<Time> duration) {
		return new RepeatableCondition() {
			long endTime;
			
			@Override
			protected void initialize() {
				endTime = System.currentTimeMillis() + (long) duration.in(Units.Milliseconds);
			}
			
			@Override
			protected boolean condition() {
				return System.currentTimeMillis() >= endTime;
			}
		};
	}
}
