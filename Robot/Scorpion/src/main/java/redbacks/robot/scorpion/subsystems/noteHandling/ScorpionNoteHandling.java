package redbacks.robot.scorpion.subsystems.noteHandling;

import static arachne4.lib.sequences.Actionable.*;

import java.util.function.Supplier;

import arachne4.lib.behaviours.Behaviour;
import arachne4.lib.logging.ArachneLogger;
import arachne4.lib.sequences.Actionable;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.CommonConstants;
import redbacks.robot.CommonScoringTargets;
import redbacks.robot.CommonScoringTargets.ScoringTarget;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.scorpion.io.ScorpionIOProvider;
import redbacks.robot.scorpion.strategies.ScorpionScoringTargets;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.Blower;
import redbacks.robot.scorpion.subsystems.noteHandling.pivot.ScorpionPivot;
import redbacks.robot.subsystems.noteHandling.CommonNoteHandling;
import redbacks.robot.subsystems.noteHandling.CommonNoteHandlingMappings;

public class ScorpionNoteHandling extends CommonNoteHandling {
    private final CommonNoteHandlingMappings mappings = scheduler.getMappings(CommonNoteHandlingMappings.class);

    private final Scorpion robot;
    public final Blower blower;

    public ScorpionNoteHandling(Scorpion robot, ScorpionIOProvider io) {
        super(robot, new ScorpionPivot(robot, io), io);

        this.robot = robot;
        this.blower = new Blower(robot, io);
    }

    @Override
    protected Behaviour createShooterSweepSpinUpBehaviour() {
        return Behaviour.thatRunsOnce(() -> {
            setShooterSpeedWithTopMultiplied(ScorpionShooterConstants.SWEEP_SPEED, 1);
            setIntakePower(0);
        });
    }

    @Override
    protected Behaviour createIntakeAndShootBehaviour() {
        return Behaviour.thatRunsOnce(() -> {
            setShooterSpeed(ScorpionShooterConstants.TREE_SPEED.get(velocityOffsetSpeakerPosition()));
            setIntakePower(ScorpionIntakeConstants.POWER);
        });
    }

    @Override
    public boolean isShooterAtSpeakerSpeed() {
        return io.inputs.rightShooterVelocity.gte(ScorpionShooterConstants.MINIMUM_SPEAKER_SPEED);
    }

    @Override
    protected ShootingSpeeds determineShootingSpeeds() {
        ScorpionNoteHandlingTarget shotMode = determineTarget();
        return new ShootingSpeeds(shotMode.shootingSpeed.apply(robot), shotMode.rightMotorSpeedMultiplier);
    }

    private ScorpionNoteHandlingTarget determineTarget() {
        ScoringTarget scoringTarget = robot.scoringTarget.get();

        if (scoringTarget == CommonScoringTargets.AMP) return ScorpionNoteHandlingTarget.AMP;
        if (scoringTarget == ScorpionScoringTargets.TRAP) return ScorpionNoteHandlingTarget.TRAP;
        if (scoringTarget == CommonScoringTargets.FEED_TO_CENTER) return ScorpionNoteHandlingTarget.FEEDING_TO_CENTRE;
        if (scoringTarget == CommonScoringTargets.FEED_TO_WING) return ScorpionNoteHandlingTarget.FEEDING_TO_AMP;

        return ScorpionNoteHandlingTarget.SPEAKER;
    }

    private double velocityOffsetSpeakerPosition() {
        return robot.getVelocityOffsetSpeakerPosition().getNorm();
    }

    private Behaviour sweepingWithDynamicSpeed(Supplier<Measure<Velocity<Distance>>> speedSupplier, boolean isRed) {
        return new Behaviour() {
            @Override
            public void onEnterMode() {
                setIntakePower(ScorpionIntakeConstants.POWER);
            }

            @Override
            public void run() {
                AutoSetShooterSpeedWithTopMultiplied(speedSupplier.get(), ScorpionShooterConstants.GENERAL_RIGHT_ROLLER_SPEED_MULTIPLER, isRed);
            }
        };
    }

    public void sweepWithDynamicSpeed(Supplier<Measure<Velocity<Distance>>> speedSupplier, boolean isRed) {
        handlingBehaviours.changeToMode(sweepingWithDynamicSpeed(speedSupplier, isRed));
    }

    private void AutoSetShooterSpeedWithTopMultiplied(Measure<Velocity<Distance>> speed, double multiplier, boolean isRed) {
        if(isRed) {
            io.setRightShooterTargetVelocity(speed);
            io.setLeftShooterTargetVelocity(speed.times(multiplier));
        } else {
            io.setRightShooterTargetVelocity(speed.times(multiplier));
            io.setLeftShooterTargetVelocity(speed);
        }
    }

    @Override
    public Actionable doShoot() {
        return SEQUENCE(
            WAIT(Units.Seconds.of(1)).UNSAFE_UNTIL((hasTimeElapsed) -> pivot.isAtAutoTarget() || hasTimeElapsed),
            DO(this::shoot),
            WAIT().UNSAFE_UNTIL(this::isShooterAtSpeakerSpeed),
            WAIT(Units.Seconds.of(0.1))
        );
    }

    { registerHandler(mappings.shoot.onPress(), this::logShotDetails); }
    private void logShotDetails() {
        var position = robot.getPosition().getTranslation();
        var shotMode = determineTarget();

        if (shotMode == ScorpionNoteHandlingTarget.SPEAKER) {
            ArachneLogger.getInstance().info(String.format(
                "Shot from (%.2f, %.2f) with shooter target speed %.1fm/s and pivot angle %.1f degrees. Distance to goal with velocity offset: %.2fm",
                position.getX(),
                position.getY(),
                shotMode.shootingSpeed.apply(robot).in(Units.MetersPerSecond),
                pivot.getAngle().getDegrees(),
                robot.getVelocityOffsetSpeakerPosition().getNorm()));
        }
        else if (shotMode == ScorpionNoteHandlingTarget.FEEDING_TO_AMP) {
            ArachneLogger.getInstance().info(String.format(
                "Feed from (%.2f, %.2f) to amp with shooter target speed %.1fm/s and pivot angle %.1f degrees. Distance to target: %.2fm",
                position.getX(),
                position.getY(),
                shotMode.shootingSpeed.apply(robot).in(Units.MetersPerSecond),
                pivot.getAngle().getDegrees(),
                position.getDistance(CommonConstants.FEED_TO_AMP_TARGET_POS.in(DriverStation.getAlliance().orElse(Alliance.Red)))));
        }
        else if (shotMode == ScorpionNoteHandlingTarget.FEEDING_TO_CENTRE) {
            ArachneLogger.getInstance().info(String.format(
                "Feed from (%.2f, %.2f) to center with shooter target speed %.1fm/s and pivot angle %.1f degrees. Distance to target: %.2fm",
                position.getX(),
                position.getY(),
                shotMode.shootingSpeed.apply(robot).in(Units.MetersPerSecond),
                pivot.getAngle().getDegrees(),
                position.getDistance(CommonConstants.FEED_TO_CENTER_TARGET_POS.in(DriverStation.getAlliance().orElse(Alliance.Red)))));
        }
    }

    @Override
    protected boolean isOnTargetToShootInDriverControlled() {
        return determineTarget().isAtSpeed.test(robot, currentSpeed());
    }

    @Override
    protected boolean isOnTargetToShootInAuto() {
        return isShooterAtSpeakerSpeed();
    }
}
