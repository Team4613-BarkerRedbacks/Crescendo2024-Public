package redbacks.robot.scorpion.subsystems.climber;

import static redbacks.robot.scorpion.subsystems.climber.ScorpionClimberConstants.*;

import arachne4.lib.behaviours.Behaviour;
import redbacks.lib.subsystems.positionbased.DistanceBasedIO;
import redbacks.lib.subsystems.positionbased.DistanceBasedSubsystem;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.scorpion.io.ScorpionIOProvider;

public class ScorpionClimber extends DistanceBasedSubsystem<DistanceBasedIO> {
    public ScorpionClimber(Scorpion robot, ScorpionIOProvider io) {
        super("Climber", robot, io.getClimberIO(), START_POSITION, ScorpionClimberVisualiser::new);

        var mappings = scheduler.getMappings(ScorpionClimberMappings.class);

        configSetManualInputSource(mappings.manualMove::get);
        configWhileManuallyMoving(input -> { if (input > 0) robot.raiseBlowerForClimbing(); });

        Behaviour raised = createMoveToTargetBehaviour(UP_POSITION)
            .alsoOnEnter(robot::raiseBlowerForClimbing);

        registerHandler(mappings.climbUpImmediately.onPress(), () -> moveTo(raised));
    }
}
