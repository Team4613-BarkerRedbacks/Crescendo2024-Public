package redbacks.robot.scorpion;

import arachne4.lib.AutoManager;
import arachne4.lib.ArachneRobotWithAdvantageKit.Mode;
import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.robot.CommonScoringTargets;
import redbacks.robot.Robot2024;
import redbacks.robot.scorpion.io.ScorpionHardware;
import redbacks.robot.scorpion.io.ScorpionIOProvider;
import redbacks.robot.scorpion.io.ScorpionSim;
import redbacks.robot.scorpion.strategies.ScorpionScoringTargets;
import redbacks.robot.scorpion.subsystems.climber.ScorpionClimber;
import redbacks.robot.scorpion.subsystems.drivetrain.ScorpionDrivetrain;
import redbacks.robot.scorpion.subsystems.noteHandling.ScorpionNoteHandling;

public class Scorpion extends Robot2024 {
    private final ScorpionMappings mappings = new ScorpionMappings(this);

    final ScorpionDrivetrain drivetrain;
    final ScorpionNoteHandling noteHandling;
    final ScorpionClimber climber;

    public Scorpion(SchedulerProvider schedulerProvider, Mode mode) {
        super(schedulerProvider);

        ScorpionIOProvider io = switch (mode) {
            case REAL -> new ScorpionHardware(scheduler, this);
            case SIMULATED -> new ScorpionSim(scheduler);
            default -> new ScorpionIOProvider.Empty(scheduler);
        };

        this.drivetrain = new ScorpionDrivetrain(this, io);
        this.noteHandling = new ScorpionNoteHandling(this, io);
        this.climber = new ScorpionClimber(this, io);

        AutoManager<Scorpion, Auto> autos = new AutoManager<>(this, Auto.DO_NOTHING, Auto.values());
        registerHandler(Scheduler.EXECUTE, autos);
        registerHandler(Scheduler.GAME_STATE_CHANGE, to(GameState.AUTO), autos::startAuto);
        registerHandler(Scheduler.GAME_STATE_CHANGE, from(GameState.AUTO), autos::stopAuto);
    }

    { registerHandler(Scheduler.EXECUTE, this::updateScoringTarget); }
    private void updateScoringTarget() {
        if (mappings.targetAmp.get()) scoringTarget.set(CommonScoringTargets.AMP);
        else if (mappings.targetFeedToCenter.get()) scoringTarget.set(CommonScoringTargets.FEED_TO_CENTER);
        else if (mappings.targetFeedToWing.get()) scoringTarget.set(CommonScoringTargets.FEED_TO_WING);
        else if (mappings.targetTrap.get()) scoringTarget.set(ScorpionScoringTargets.TRAP);
        else scoringTarget.set(CommonScoringTargets.SPEAKER);
    }

    { registerHandler(Scheduler.POST_EXECUTE, this::telemetry); }
    private void telemetry() {
        SmartDashboard.putString("Scoring target", scoringTarget.get().toString());
    }

    public void raiseBlowerForClimbing() {
        noteHandling.blower.raiseForClimbing();
    }

    @Override
    public Pose2d getPosition() {
        return drivetrain.getPosition();
    }

    @Override
    public ChassisSpeeds getFieldRelativeVelocity() {
        return drivetrain.getFieldRelativeVelocity();
    }

    public Translation2d getVelocityOffsetSpeakerPosition() {
        return drivetrain.getVelocityOffsetSpeakerPosition();
    }
}