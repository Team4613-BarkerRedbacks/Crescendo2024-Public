
package redbacks.robot.scorpion.subsystems.noteHandling.blower;

import static redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerConstants.*;

import arachne4.lib.behaviours.Behaviour;
import redbacks.lib.subsystems.positionbased.AngleBasedSubsystem;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.scorpion.io.ScorpionIOProvider;
import redbacks.robot.scorpion.strategies.ScorpionScoringTargets;

public class Blower extends AngleBasedSubsystem<BlowerIO> {
    private final Behaviour climbing = createMoveToTargetBehaviour(CLIMB_ANGLE);

    public Blower(Scorpion robot, ScorpionIOProvider io) {
        super("Blower", robot, io.getBlowerIO(), ROTATION_MINUMUM, BlowerVisualiser::new);

        configSetMinimumPosition(ROTATION_MINUMUM);
        configSetMaximumPosition(ROTATION_MAXIMUM);

        Behaviour blowingTrap = createMoveToTargetBehaviour(BLOW_ANGLE)
            .alsoOnEnter(() -> blow(true))
            .alsoOnLeave(() -> blow(false));

        Behaviour stowed = createMoveToTargetBehaviour(STOW_ANGLE);

        registerHandler(robot.scoringTarget.onChange(), change -> {
            if (change.to == ScorpionScoringTargets.TRAP) moveTo(blowingTrap);
            else if (change.from == ScorpionScoringTargets.TRAP) moveTo(stowed);
        });
    }

    public void raiseForClimbing() {
        moveTo(climbing);
    }

    private void blow(boolean shouldBlow) {
        io.blowWithPercentageOutput(shouldBlow ? BLOW_POWER : 0);
    }
}
