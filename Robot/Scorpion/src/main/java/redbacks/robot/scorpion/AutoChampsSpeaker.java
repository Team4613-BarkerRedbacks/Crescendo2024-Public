package redbacks.robot.scorpion;

import static arachne4.lib.sequences.Actionable.*;
import static redbacks.field.FieldLocations.*;
import static redbacks.robot.subsystems.drivetrain.HeadingControlUtils.*;

import java.util.function.Function;

import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.sequences.Actionable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.scorpion.subsystems.drivetrain.ScorpionDrivetrainConstants;

public class AutoChampsSpeaker {

    // ----------------------------------------
    // Constants
    // ----------------------------------------

    // ----- Initial move to score preload and middle wing note -----

    private static final AllianceSpecific<Pose2d> STARTING_POS = AllianceSpecific
        .forRed(new Pose2d(
            EDGE_OF_SUBWOOFER_X.plus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters),
            WING_NOTE_2.in(Alliance.Red).getY() + 0.17,
            Rotation2d.fromDegrees(0)))
        .forBlue(new Pose2d(
            EDGE_OF_SUBWOOFER_X.plus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters),
            WING_NOTE_2.in(Alliance.Blue).getY() - 0.17,
            Rotation2d.fromDegrees(0)));

    private static final AllianceSpecific<Translation2d> SHOT_1 = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_SUBWOOFER_X.in(Units.Meters) + 1.75, WING_NOTE_2.in(Alliance.Red).getY()))
        .forBlue(new Translation2d(EDGE_OF_SUBWOOFER_X.in(Units.Meters) + 1.75, WING_NOTE_2.in(Alliance.Blue).getY()));

    // ----- Intake and score centreline note 2 -----

    private static final AllianceSpecific<Translation2d> CENTERNOTE_2 = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters) + 0.35, CENTER_NOTE_2.in(Alliance.Red).getY() + 0.1))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters) + 0.35, CENTER_NOTE_2.in(Alliance.Blue).getY() - 0.1));

    private static final AllianceSpecific<Translation2d> SHOT_2 = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.1, 1.4))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.1, -1.4));

    // ----- Intake and score centreline note 3 -----

    private static final AllianceSpecific<Translation2d> PRE_CENTERNOTE_3 = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.1, FIELD_WIDTH.in(Units.Meters) / 2))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.1, -FIELD_WIDTH.in(Units.Meters) / 2));
    
    private static final AllianceSpecific<Translation2d> CENTERNOTE_3 = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters) + 0.3, CENTER_NOTE_3.in(Alliance.Red).getY() - 0.05))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters) + 0.3, CENTER_NOTE_3.in(Alliance.Blue).getY() + 0.05));

    // ----- Intake and score the two last wing notes-----

    private static final AllianceSpecific<Translation2d> PRE_WING = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_SUBWOOFER_X.in(Units.Meters) + 1.1, WING_NOTE_2.in(Alliance.Red).getY()))
        .forBlue(new Translation2d(EDGE_OF_SUBWOOFER_X.in(Units.Meters) + 1.1, WING_NOTE_2.in(Alliance.Blue).getY()));

    private static final AllianceSpecific<Translation2d> WING_3 = AllianceSpecific
        .forRed(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.15,
            WING_NOTE_3.in(Alliance.Red).getY() - ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) + 0.2))
        .forBlue(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.15,
            WING_NOTE_3.in(Alliance.Blue).getY() + ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) - 0.2));

    private static final AllianceSpecific<Translation2d> WING_1 = AllianceSpecific
        .forRed(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.25,
            WING_NOTE_1.in(Alliance.Red).getY() - ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) + 0.05))
        .forBlue(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.25,
            WING_NOTE_1.in(Alliance.Blue).getY() + ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) - 0.05));
        
    private static final AllianceSpecific<Translation2d> WING_3_WAYPOINT = AllianceSpecific
        .forRed(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) + 0.15,
            WING_NOTE_3.in(Alliance.Red).getY() - ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) + 0.2))
        .forBlue(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) + 0.15,
            WING_NOTE_3.in(Alliance.Blue).getY() + ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) - 0.2));

    private static final AllianceSpecific<Translation2d> WING_1_WAYPOINT = AllianceSpecific
        .forRed(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.2,
            WING_NOTE_1.in(Alliance.Red).getY() - ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) + 0.35))
        .forBlue(new Translation2d(
            WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.2,
            WING_NOTE_1.in(Alliance.Blue).getY() + ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE.in(Units.Meters) - 0.35));

    // ----- Pivot angles -----

    private static final AllianceSpecific<Rotation2d> CENTRENOTE_3_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(50.7))
        .forBlue(Rotation2d.fromDegrees(50.7));

    private static final AllianceSpecific<Rotation2d> PRELOAD_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(44))
        .forBlue(Rotation2d.fromDegrees(44));

    private static final AllianceSpecific<Rotation2d> CENTRENOTE_2_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(29.6))
        .forBlue(Rotation2d.fromDegrees(29.6));

    private static final AllianceSpecific<Rotation2d> WING_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(37.4))
        .forBlue(Rotation2d.fromDegrees(37.4));

    // ----------------------------------------
    // Autonomous definition
    // ----------------------------------------

    public static Function<Scorpion, Actionable> createForAlliance(Alliance alliance) {
        return (robot) -> SEQUENCE(
            DO(() -> robot.drivetrain.setPosition(STARTING_POS.in(alliance))),
            DO(robot.noteHandling::shooterSpinUp),
            DO (() ->robot.noteHandling.moveToAngle(PRELOAD_ANGLE.in(alliance))),
            // Preload and middle wing note
            SPLIT(
                robot.drivetrain
                    .doMoveTo(SHOT_1.in(alliance))
                    .withHeading(Rotation2d.fromDegrees(0))
                    .withEndVelocity(Units.MetersPerSecond.of(0.25))
                    .precomputedWithStartPosition(STARTING_POS.in(alliance).getTranslation())      
            ).AND(
                SEQUENCE(
                    WAIT().UNSAFE_UNTIL(() -> robot.drivetrain.getPosition().getX() > SHOT_1.in(alliance).getX() - 0.6),
                    DO(robot.noteHandling::intakeShoot)
                )
            ),
            WAIT(Units.Seconds.of(0.3)),
        // Centreline note 2
            robot.drivetrain
                .doMoveTo(SHOT_2.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .withEndVelocity(Units.MetersPerSecond.of(0.3))
                .precomputedFromLastTarget(),
            DO (() ->robot.noteHandling.moveToAngle(CENTRENOTE_2_ANGLE.in(alliance))),
            DO(robot.noteHandling::intake),
            robot.drivetrain
                .doMoveTo(CENTERNOTE_2.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(SHOT_2.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
        // Centreline note 3
            DO(robot.noteHandling::intake),
            robot.drivetrain
                .doMoveTo(PRE_CENTERNOTE_3.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),
            DO (() -> robot.noteHandling.moveToAngle(CENTRENOTE_3_ANGLE.in(alliance))),  
            robot.drivetrain
                .doMoveTo(CENTERNOTE_3.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),          
            robot.drivetrain
                .doMoveTo(PRE_WING.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
        // Last two wing notes
            DO(robot.noteHandling::intakeWithShooterAtSpeed),
            DO (() -> robot.noteHandling.moveToAngle(WING_ANGLE.in(alliance))),
            robot.drivetrain
                .doMoveTo(WING_3.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(WING_3_WAYPOINT.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            DO(robot.noteHandling::intakeWithShooterAtSpeed),
            robot.drivetrain
                .doMoveTo(WING_1_WAYPOINT.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .withEndVelocity(Units.MetersPerSecond.of(0.25))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(WING_1.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .withEndVelocity(Units.MetersPerSecond.of(0.25))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot()
        );
    }
}
