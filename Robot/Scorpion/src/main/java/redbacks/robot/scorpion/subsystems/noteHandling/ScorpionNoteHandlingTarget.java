package redbacks.robot.scorpion.subsystems.noteHandling;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.CommonConstants;
import redbacks.robot.scorpion.Scorpion;

import static redbacks.robot.scorpion.subsystems.noteHandling.ScorpionShooterConstants.*;

import java.util.function.BiPredicate;
import java.util.function.Function;

public enum ScorpionNoteHandlingTarget {
    SPEAKER(ScorpionNoteHandlingTarget::speakerShootingSpeed, GENERAL_RIGHT_ROLLER_SPEED_MULTIPLER, (robot, currentSpeed) -> currentSpeed.gte(ScorpionShooterConstants.TREE_SPEED.get(robot.getVelocityOffsetSpeakerPosition().getNorm()).minus(THRESHOLD_FOR_FEEDING))),
    AMP((robot) -> AMP_SPEED, 1, (robot, currentSpeed) -> currentSpeed.lte(MAXIMUM_AMP_SPEED)),
    FEEDING_TO_CENTRE((robot) -> feedingCentreSpeed(robot, CommonConstants.FEED_TO_CENTER_TARGET_POS.in(DriverStation.getAlliance().orElse(Alliance.Red))), 0.9, (robot, currentSpeed) -> true), 
    FEEDING_TO_AMP((robot) -> feedingAmpSpeed(robot, CommonConstants.FEED_TO_AMP_TARGET_POS.in(DriverStation.getAlliance().orElse(Alliance.Red))), 0.9, (robot, currentSpeed) -> currentSpeed.gte(ScorpionShooterConstants.FEED_AMP_TREE_SPEED.get(robot.getPosition().getTranslation().getNorm()).minus(THRESHOLD_FOR_FEEDING))),
    TRAP((robot) -> TRAP_SPEED, 1, (robot, currentSpeed) -> currentSpeed.gte(Units.MetersPerSecond.of(9)) && currentSpeed.lte(Units.MetersPerSecond.of(10.5)));

    public final Function<Scorpion, Measure<Velocity<Distance>>> shootingSpeed;
    public final double rightMotorSpeedMultiplier;
    public final BiPredicate<Scorpion, Measure<Velocity<Distance>>> isAtSpeed;

    ScorpionNoteHandlingTarget(Function<Scorpion, Measure<Velocity<Distance>>> shootingSpeed, double rightMotorSpeedMultiplier, BiPredicate<Scorpion, Measure<Velocity<Distance>>> isAtSpeed) {
        this.shootingSpeed = shootingSpeed;
        this.rightMotorSpeedMultiplier = rightMotorSpeedMultiplier;
        this.isAtSpeed = isAtSpeed;
    }

    public static Measure<Velocity<Distance>> speakerShootingSpeed(Scorpion robot) {
        double currentPos = robot.getVelocityOffsetSpeakerPosition().getNorm();
        return TREE_SPEED.get(currentPos);
    }

    public static Measure<Velocity<Distance>> feedingCentreSpeed(Scorpion robot, Translation2d targetPos) {
        Translation2d currentPos = robot.getPosition().getTranslation();
        Translation2d distance = currentPos.minus(targetPos);
        return FEED_CENTRE_TREE_SPEED.get(distance.getNorm());
    }

    public static Measure<Velocity<Distance>> feedingAmpSpeed(Scorpion robot, Translation2d targetPos) {
        Translation2d currentPos = robot.getPosition().getTranslation();
        Translation2d distance = currentPos.minus(targetPos);
        return FEED_AMP_TREE_SPEED.get(distance.getNorm());
    }
}
