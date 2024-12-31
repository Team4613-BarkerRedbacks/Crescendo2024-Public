package arachne4.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

public class ArachneMath {
	public static final Rotation2d
		DEGREES_0 = new Rotation2d(),
		DEGREES_180 = Rotation2d.fromRadians(Math.PI),
		DEGREES_90 = Rotation2d.fromDegrees(90),
		DEGREES_MINUS_90 = Rotation2d.fromDegrees(-90);

	public static final Measure<Distance> DISTANCE_0 = Units.Meters.zero();

	public static double signedPow(double value, double exponent) {
		if(value < 0) return -Math.pow(-value, exponent);
		return Math.pow(value, exponent);
	}

    public static double getClosestEquivalentAngle(double value, double reference, Angle unit) {
		double halfRotation = Units.Rotation.of(0.5).in(unit);
		return MathUtil.inputModulus(value, reference - halfRotation, reference + halfRotation);
    }
}
