package redbacks.robot.scorpion.subsystems.noteHandling.pivot;

import static redbacks.robot.scorpion.subsystems.noteHandling.pivot.ScorpionPivotConstants.*;

import arachne4.lib.behaviours.Behaviour;
import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.robot.CommonScoringTargets;
import redbacks.robot.CommonScoringTargets.ScoringTarget;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.scorpion.io.ScorpionIOProvider;
import redbacks.robot.scorpion.strategies.ScorpionScoringTargets;
import redbacks.robot.subsystems.noteHandling.pivot.CommonPivot;

public class ScorpionPivot extends CommonPivot {
    private final Scorpion robot;
    private final Behaviour goToTrapPosition;

    public ScorpionPivot(Scorpion robot, ScorpionIOProvider io) {
        super(robot, io);

        this.robot = robot;
        this.goToTrapPosition = createMoveToTargetBehaviour(TRAP_ANGLE);
    }

    @Override
    public boolean isAtAutoTarget() {
        return isAtTarget();
    }

    @Override
    public boolean isAtTeleopTarget() {
        return isAtTarget();
    }

    public boolean isAtTarget() {
        return Math.abs(inputs.currentPosition.getDegrees() - inputs.targetPosition.getDegrees()) < TARGET_TOLERANCE;
    }

    @Override
    protected Behaviour chooseBehaviourFromScoringTarget(ScoringTarget scoringTarget, AimingStrategy aimingStrategy) {
        if (scoringTarget == CommonScoringTargets.SPEAKER) {
            return aimingStrategy == AimingStrategy.DYNAMIC ? automaticAiming : goToSubwooferPosition;
        }
        else if (scoringTarget == CommonScoringTargets.AMP) {
            return goToAmpPosition;
        }
        else if (scoringTarget == CommonScoringTargets.FEED_TO_CENTER) {
            return goToFeedCentre;
        }
        else if (scoringTarget == CommonScoringTargets.FEED_TO_WING) {
            return goToFeedAmp;
        }
        else if (scoringTarget == ScorpionScoringTargets.TRAP) {
            return goToTrapPosition;
        }

        return null;
    }

    { registerHandler(Scheduler.POST_EXECUTE, this::telemetry); }
    private void telemetry() {
        SmartDashboard.putBoolean("is Pivot At Target", isAtTarget());
    }

    @Override
    protected Behaviour createAutomaticAimingBehaviour() {
        return createDynamicFollowTargetBehaviour(
            () -> DISTANCE_TO_ANGLE_MAP.in(DriverStation.getAlliance().orElse(Alliance.Red)).get(robot.getVelocityOffsetSpeakerPosition().getNorm()));
    }
}
