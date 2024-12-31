package redbacks.robot.scorpion.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import arachne4.lib.color.ColorUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import redbacks.lib.subsystems.positionbased.PositionVisualiser;

public class ScorpionClimberVisualiser implements PositionVisualiser<Measure<Distance>> {
    private static final Translation3d COR_POSE_OFFSET = new Translation3d(0.3075, 0, 0.569);

    // Y corresponds to the Z-axis (up positive)
    private static final Translation2d
        PIVOT_POINT = new Translation2d(COR_POSE_OFFSET.getX(), COR_POSE_OFFSET.getZ()),
        WINCH_POINT = new Translation2d(0.262, 0.140 + 0.035);

    private static final Measure<Distance>
        ARM_LENGTH_TO_MEASUREMENT_POINT = Units.Meters.of(0.4),
        ROPE_ATTACH_LENGTH_ALONG_ARM = Units.Meters.of(0.130);

    private static final double ABS_ANGLE_OF_LINE_BETWEEN_PIVOT_AND_WINCH_RADS = Math.PI + WINCH_POINT.minus(PIVOT_POINT).getAngle().getRadians();
    private static final double DISTANCE_FROM_WINCH_TO_PIVOT_METERS = WINCH_POINT.getDistance(PIVOT_POINT);

    private final String visualiserName;

    private final Mechanism2d mechanism = new Mechanism2d(1.7, 1, new Color8Bit(Color.kGray));
    private final MechanismRoot2d centerOfRotation = mechanism.getRoot("CoR", 0.85 + PIVOT_POINT.getX(), PIVOT_POINT.getY());
    private final MechanismRoot2d winch = mechanism.getRoot("winch", 0.85 + WINCH_POINT.getX(), WINCH_POINT.getY());
    private final MechanismLigament2d climber, rope;

    /**
     * Create a new visualiser for the climber position.
     * This logs both a Mechanism2d and a Pose3d for display in AdvantageScope.
     * 
     * @param visualiserName A descriptor for this visualiser, e.g. "Actual" or "Target"
     * @param initialHeight The initial climber height to display
     */
    public ScorpionClimberVisualiser(String visualiserName, Measure<Distance> initialHeight, Color color) {
        this.visualiserName = visualiserName;

        double initialArmAngleRads = getArmAngleRadsFromHeight(initialHeight);
        this.climber = centerOfRotation.append(new MechanismLigament2d(
            "Climber",
            ARM_LENGTH_TO_MEASUREMENT_POINT.in(Units.Meters),
            180 - Math.toDegrees(initialArmAngleRads),
            10,
            new Color8Bit(color)));

        Translation2d initialRopeVector = getRopeVectorFromArmAngle(initialArmAngleRads);
        this.rope = winch.append(new MechanismLigament2d(
            "Rope",
            initialRopeVector.getNorm(),
            initialRopeVector.getAngle().getDegrees(),
            5,
            new Color8Bit(ColorUtil.shift(color, 0, 0.3, 0.8))));
    }

    @Override
    public void update(Measure<Distance> height) {
        double angleRads = getArmAngleRadsFromHeight(height);
        climber.setAngle(180 - Math.toDegrees(angleRads));

        Translation2d ropeVector = getRopeVectorFromArmAngle(angleRads);
        rope.setAngle(ropeVector.getAngle().getDegrees());
        rope.setLength(ropeVector.getNorm());

        Logger.recordOutput(String.format("Climber/%s/Angle", visualiserName), angleRads);
        Logger.recordOutput(String.format("Climber/%s/Mechanism", visualiserName), mechanism);
        Logger.recordOutput(String.format("Climber/%s/Pose3d", visualiserName), new Pose3d(COR_POSE_OFFSET, new Rotation3d(0, angleRads, 0)));
    }

    private static final Translation2d getRopeVectorFromArmAngle(double armAngleRads) {
        return new Translation2d(ROPE_ATTACH_LENGTH_ALONG_ARM, Units.Meters.zero())
            .rotateBy(Rotation2d.fromRadians(Math.PI - armAngleRads))
            .plus(PIVOT_POINT)
            .minus(WINCH_POINT);
    }

    private static final double getArmAngleRadsFromHeight(Measure<Distance> height) {
        double heightAbovePivot = height.in(Units.Meters) - PIVOT_POINT.getY();
        return Math.asin(heightAbovePivot / ARM_LENGTH_TO_MEASUREMENT_POINT.in(Units.Meters));
    }

    private static final Measure<Distance> getRopeLengthForHeight(Measure<Distance> height) {
        double oppositeAngleRads = getArmAngleRadsFromHeight(height) + ABS_ANGLE_OF_LINE_BETWEEN_PIVOT_AND_WINCH_RADS;
        double a = DISTANCE_FROM_WINCH_TO_PIVOT_METERS;
        double b = ROPE_ATTACH_LENGTH_ALONG_ARM.in(Units.Meters);

        return Units.Meters.of(Math.sqrt(a*a + b*b - 2*a*b*Math.cos(oppositeAngleRads)));
    }
}
