package redbacks.robot.subsystems.noteHandling.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import redbacks.lib.subsystems.positionbased.PositionVisualiser;

public class PivotVisualiser implements PositionVisualiser<Rotation2d> {
    private static final Translation3d COR_POSE_OFFSET = new Translation3d(0.017, 0, 0.19);
    private static final double SHOOTER_VISUAL_LENGTH = 0.3;

    private final String visualiserName;

    private final Mechanism2d mechanism = new Mechanism2d(1.7, 1, new Color8Bit(Color.kGray));
    private final MechanismRoot2d centerOfRotation = mechanism.getRoot("CoR", 0.85, 0.2);
    private final MechanismLigament2d shooter;

    /**
     * Create a new visualiser for the pivot position.
     * This logs both a Mechanism2d and a Pose3d for display in AdvantageScope.
     * 
     * @param visualiserName A descriptor for this visualiser, e.g. "Actual" or "Target"
     * @param initialAngleRads The initial pivot angle to display
     */
    public PivotVisualiser(String visualiserName, Rotation2d initialAngle, Color color) {
        this.visualiserName = visualiserName;

        this.shooter = centerOfRotation.append(new MechanismLigament2d(
            "Shooter",
            SHOOTER_VISUAL_LENGTH,
            180 - initialAngle.getDegrees(),
            10,
            new Color8Bit(color)));
    }

    @Override
    public void update(Rotation2d angle) {
        shooter.setAngle(180 - angle.getDegrees());

        Logger.recordOutput(String.format("Pivot/%s/Mechanism", visualiserName), mechanism);
        Logger.recordOutput(String.format("Pivot/%s/Pose3d", visualiserName), new Pose3d(COR_POSE_OFFSET, new Rotation3d(0, angle.getRadians(), 0)));
    }
}
