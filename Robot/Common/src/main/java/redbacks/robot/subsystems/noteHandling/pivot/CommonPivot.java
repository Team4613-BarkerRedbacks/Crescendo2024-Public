package redbacks.robot.subsystems.noteHandling.pivot;

import arachne4.lib.Constants;
import arachne4.lib.behaviours.Behaviour;
import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.lib.subsystems.positionbased.AngleBasedIO;
import redbacks.lib.subsystems.positionbased.AngleBasedSubsystem;
import redbacks.robot.CommonScoringTargets.ScoringTarget;
import redbacks.robot.io.IOProvider;
import redbacks.robot.Robot2024;

public abstract class CommonPivot extends AngleBasedSubsystem<AngleBasedIO> {
    private static final PivotConstants CONSTANTS = Constants.get(PivotConstants.class);
    private final PivotMappings mappings = scheduler.getMappings(PivotMappings.class);

    // ----------------------------------------
    // Subsystem Definition
    // ----------------------------------------

    protected static enum AimingStrategy {
        DYNAMIC,
        FIXED,
        MANUAL;
    }

    private final Robot2024 robot;
    protected final Behaviour automaticAiming, goToFeedCentre, goToFeedAmp;
    protected final Behaviour goToSubwooferPosition, goToAmpPosition, climbing;

    private AimingStrategy aimingStrategy = AimingStrategy.DYNAMIC;

    public CommonPivot(Robot2024 robot, IOProvider io) {
        super("Pivot", robot, io.getPivotIO(), CONSTANTS.hardStopAngle(), PivotVisualiser::new);

        this.robot = robot;

        configSetMinimumPosition(CONSTANTS.minimumAngle());
        configSetMaximumPosition(CONSTANTS.maximumAngle());

        configSetManualInputSource(mappings.manualMove::get);
        configSetApplyPositionLimitsInManualMode(CONSTANTS.manualEndpointRange());

        // Dynamic targets
        automaticAiming = createAutomaticAimingBehaviour();

        goToFeedCentre = createDynamicFollowTargetBehaviour(
            () -> CONSTANTS.feedToCenterAnglesFromXVelocity().get(robot.getFieldRelativeVelocity().vxMetersPerSecond));

        goToFeedAmp = createDynamicFollowTargetBehaviour(
            () -> CONSTANTS.feedToAmpAnglesFromXVelocity().get(robot.getFieldRelativeVelocity().vxMetersPerSecond));

        // Fixed positions
        goToSubwooferPosition = createMoveToTargetBehaviour(CONSTANTS.subwooferAngle());
        goToAmpPosition = createMoveToTargetBehaviour(CONSTANTS.ampAngle());
        climbing = createMoveToTargetBehaviour(CONSTANTS.lowAngle());

        moveTo(automaticAiming);
    }

    protected abstract Behaviour createAutomaticAimingBehaviour();

    // ----------------------------------------
    // Public State-Modifiers & Accessors
    // ----------------------------------------

    public Rotation2d getAngle() {
        return inputs.currentPosition;
    }

    public abstract boolean isAtAutoTarget();
    public abstract boolean isAtTeleopTarget();

    // ----------------------------------------
    // Scheduled Events
    // ----------------------------------------

    { registerHandler(Scheduler.POST_EXECUTE, this::telemetry); }
    private void telemetry() {
        SmartDashboard.putString("Aiming strategy", aimingStrategy.name());
    }

    // ----- Controls -----

    { registerHandler(Scheduler.EXECUTE, GameState.DRIVER_CONTROLLED_STATES, this::manageAimingStrategy); }
    private void manageAimingStrategy() {
        if (mappings.autoAim.get()) {
            aimingStrategy = AimingStrategy.DYNAMIC;
        }
        else if (mappings.fixedAim.get()) {
            aimingStrategy = AimingStrategy.FIXED;
        }
        else if (mappings.manualMove.get() != 0) {
            aimingStrategy = AimingStrategy.MANUAL;
        }
    }

    { registerHandler(Scheduler.EXECUTE, GameState.DRIVER_CONTROLLED_STATES, this::manageAimingBehaviours); }
    private void manageAimingBehaviours() {
        if (aimingStrategy == AimingStrategy.MANUAL) {
            changeToManualMode();
            return;
        }

        ScoringTarget scoringTarget = robot.scoringTarget.get();

        if (mappings.aimToClimb.get()) {
            moveTo(climbing);
        }
        else {
            Behaviour behaviourFromScoringTarget = chooseBehaviourFromScoringTarget(scoringTarget, aimingStrategy);
            if (behaviourFromScoringTarget != null) moveTo(behaviourFromScoringTarget);
        }
    }

    protected abstract Behaviour chooseBehaviourFromScoringTarget(ScoringTarget scoringTarget, AimingStrategy aimingStrategy);
}
