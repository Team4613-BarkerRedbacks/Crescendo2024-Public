package redbacks.robot.scorpion;

import static arachne4.lib.sequences.Actionable.*;
import static redbacks.field.FieldLocations.*;
import static redbacks.robot.subsystems.drivetrain.HeadingControlUtils.*;

import java.util.Map;
import java.util.function.Function;

import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.sequences.Actionable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.scorpion.subsystems.drivetrain.ScorpionDrivetrainConstants;

public class AutoDduCenter {

    // ----------------------------------------
    // Constants
    // ----------------------------------------

    // ----- Positions -----

    private static final AllianceSpecific<Pose2d> STARTING_POS = AllianceSpecific
        .forRed(new Pose2d(
            EDGE_OF_SUBWOOFER_X.plus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters),
            WING_NOTE_2.in(Alliance.Red).getY() + 0.17,
            Rotation2d.fromDegrees(0)))
        .forBlue(new Pose2d(
            EDGE_OF_SUBWOOFER_X.plus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters),
            WING_NOTE_2.in(Alliance.Blue).getY() - 0.17,
            Rotation2d.fromDegrees(0)));

    private static final AllianceSpecific<Translation2d> FIRST_CENTER_SHOOTING_POS = AllianceSpecific
        .forRed(new Translation2d(4, 1.2))
        .forBlue(new Translation2d(4, -1.2));

    private static final AllianceSpecific<Translation2d> WAYPOINT_TO_CENTER_NOTE_3 = AllianceSpecific
        .forRed(new Translation2d(6.5, 1.6))
        .forBlue(new Translation2d(6.5, -1.6));

    // ----- Position offsets -----

    private static final AllianceSpecific<Translation2d> PICKUP_OFFSET_FOR_WING_NOTE_3 = AllianceSpecific
        .forRed(new Translation2d(0.3, 0))
        .forBlue(new Translation2d(0.2, 0));

    private static final AllianceSpecific<Translation2d> PICKUP_OFFSET_FOR_WING_NOTE_2 = AllianceSpecific
        .forRed(new Translation2d(0.2, 0.15))
        .forBlue(new Translation2d(0.2, -0.15));

    private static final AllianceSpecific<Translation2d> PICKUP_OFFSET_FOR_WING_NOTE_1 = AllianceSpecific
        .forRed(new Translation2d(0.45, 0.3))
        .forBlue(new Translation2d(0.3, -0.3));

    private static final AllianceSpecific<Translation2d> SHOOT_OFFSET_FOR_WING_NOTE_1 = AllianceSpecific
        .forRed(new Translation2d(0, 0.5))
        .forBlue(new Translation2d(0, -0.5));

    private static final AllianceSpecific<Translation2d> PICKUP_OFFSET_FOR_CENTER_NOTE_1 = AllianceSpecific
        .forRed(new Translation2d(0.3, 0.25))
        .forBlue(new Translation2d(0.3, -0.2));

    private static final AllianceSpecific<Translation2d> PICKUP_OFFSET_FOR_CENTER_NOTE_2 = AllianceSpecific
        .forRed(new Translation2d(0.3, 0.3))
        .forBlue(new Translation2d(0.3, 0));

    private static final AllianceSpecific<Translation2d> PICKUP_OFFSET_FOR_CENTER_NOTE_3 = AllianceSpecific
        .forRed(new Translation2d(0.3, 0))
        .forBlue(new Translation2d(0.3, 0));

    private static final AllianceSpecific<Translation2d> SHOT_TARGET_OFFSET = AllianceSpecific
        .forRed(new Translation2d(0, 0))
        .forBlue(new Translation2d(0, -0.5));

    // ----- Pivot angles -----

    private static final AllianceSpecific<Rotation2d> PRELOAD_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(63))
        .forBlue(Rotation2d.fromDegrees(63));

    // ----------------------------------------
    // Autonomous definition
    // ----------------------------------------

    public static Function<Scorpion, Actionable> createForAlliance(Alliance alliance) {
        return (robot) -> SEQUENCE(
            DO(() -> robot.drivetrain.setPosition(STARTING_POS.in(alliance))),
            DO(robot.noteHandling::shooterSpinUp),
            DO (() -> robot.noteHandling.moveToAngle(PRELOAD_ANGLE.in(alliance))),
            WAIT().UNSAFE_UNTIL(() -> robot.noteHandling.getPivotAngle().getDegrees() > PRELOAD_ANGLE.in(alliance).getDegrees() - 3 && robot.noteHandling.isShooterAtSpeakerSpeed()),
            DO(robot.noteHandling::shoot),
            WAIT(Units.Seconds.of(0.1)),
            // 3 wing notes
            DO(() -> robot.noteHandling.moveToAngle(AllianceSpecific
                .forRed(Rotation2d.fromDegrees(40))
                .forBlue(Rotation2d.fromDegrees(39))
                .in(alliance))),
            SPLIT(
                robot.drivetrain
                    .doMoveTo(WING_NOTE_3.in(alliance)
                        .minus(new Translation2d(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE, Units.Meters.zero()))
                        .plus(PICKUP_OFFSET_FOR_WING_NOTE_3.in(alliance)))
                    .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                    .precomputedWithStartPosition(STARTING_POS.in(alliance).getTranslation())
            ).AND(
                robot.noteHandling.waitForNotePassthrough(Units.Seconds.of(2))
            ),
            DO(() -> robot.noteHandling.moveToAngle(AllianceSpecific
                .forRed(Rotation2d.fromDegrees(42))
                .forBlue(Rotation2d.fromDegrees(42))
                .in(alliance))),
            SPLIT(
                robot.drivetrain
                    .doMoveTo(WING_NOTE_2.in(alliance).plus(PICKUP_OFFSET_FOR_WING_NOTE_2.in(alliance)))
                    .withWaypoints(new Translation2d(
                        WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.5,
                        WING_NOTE_2.in(alliance).getY()))
                    .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                    .precomputedFromLastTarget()
            ).AND(
                robot.noteHandling.waitForNotePassthrough(Units.Seconds.of(2))
            ),
            DO(() -> robot.noteHandling.moveToAngle(AllianceSpecific
                .forRed(Rotation2d.fromDegrees(40))
                .forBlue(Rotation2d.fromDegrees(40))
                .in(alliance))),
            SPLIT(
                robot.drivetrain
                    .doMoveTo(WING_NOTE_1.in(alliance).plus(PICKUP_OFFSET_FOR_WING_NOTE_1.in(alliance)))
                    .withWaypoints(new Translation2d(
                        WING_NOTE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE).in(Units.Meters) - 0.5,
                        (WING_NOTE_1.in(alliance).getY() + WING_NOTE_2.in(alliance).getY()) / 2))
                    .withHeading(aimAtPoint(SPEAKER.in(alliance).plus(SHOOT_OFFSET_FOR_WING_NOTE_1.in(alliance)), false) )
                    .precomputedFromLastTarget()
            ).AND(
                robot.noteHandling.waitForNotePassthrough(Units.Seconds.of(2))
            ),
            DO(robot.noteHandling::intake),
            // Center note 1
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_1.in(alliance).plus(PICKUP_OFFSET_FOR_CENTER_NOTE_1.in(alliance)))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            DO(() -> robot.noteHandling.moveToAngle(AllianceSpecific
                .forRed(Rotation2d.fromDegrees(32))
                .forBlue(Rotation2d.fromDegrees(32))
                .in(alliance))),
            robot.drivetrain
                .doMoveTo(FIRST_CENTER_SHOOTING_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance).plus(SHOT_TARGET_OFFSET.in(alliance)), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            WAIT(Units.Seconds.of(0.1)),
            DO(robot.noteHandling::intake),
            // Center note 2
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_2.in(alliance).plus(PICKUP_OFFSET_FOR_CENTER_NOTE_2.in(alliance)))
                .withHeading(interpolateHeadingOnX(Map.of(
                    FIRST_CENTER_SHOOTING_POS.in(alliance), FIRST_CENTER_SHOOTING_POS.in(alliance).minus(SPEAKER.in(alliance)).getAngle(),
                    CENTER_NOTE_2.in(alliance), CENTER_NOTE_2.in(alliance).minus(FIRST_CENTER_SHOOTING_POS.in(alliance)).getAngle()
                )))
                .precomputedFromLastTarget(),
            DO(() -> robot.noteHandling.moveToAngle(AllianceSpecific
                .forRed(Rotation2d.fromDegrees(31.5))
                .forBlue(Rotation2d.fromDegrees(32))
                .in(alliance))),
            robot.drivetrain
                .doMoveTo(FIRST_CENTER_SHOOTING_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance).plus(SHOT_TARGET_OFFSET.in(alliance)), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            WAIT(Units.Seconds.of(0.1)),
            DO(robot.noteHandling::intake),
            // Center note 3
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_3.in(alliance).plus(PICKUP_OFFSET_FOR_CENTER_NOTE_3.in(alliance)))
                .withWaypoints(WAYPOINT_TO_CENTER_NOTE_3.in(alliance))
                .withHeading(interpolateHeadingOnX(Map.of(
                    FIRST_CENTER_SHOOTING_POS.in(alliance), FIRST_CENTER_SHOOTING_POS.in(alliance).minus(SPEAKER.in(alliance)).getAngle(),
                    CENTER_NOTE_3.in(alliance), CENTER_NOTE_3.in(alliance).minus(WAYPOINT_TO_CENTER_NOTE_3.in(alliance)).getAngle()
                )))
                .precomputedFromLastTarget(),
            DO(() -> robot.noteHandling.moveToAngle(AllianceSpecific
                .forRed(Rotation2d.fromDegrees(30.5))
                .forBlue(Rotation2d.fromDegrees(31))
                .in(alliance))),
            robot.drivetrain
                .doMoveTo(AllianceSpecific
                    .forRed(new Translation2d(4.5, 3.6))
                    .forBlue(new Translation2d(4.5, -3.6))
                    .in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot()
        );
    }
}
