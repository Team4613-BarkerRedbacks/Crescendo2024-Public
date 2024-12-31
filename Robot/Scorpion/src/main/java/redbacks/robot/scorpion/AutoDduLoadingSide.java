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
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoDduLoadingSide {
    // ----------------------------------------
    // Constants
    // ----------------------------------------

    // ----- Initial move to centerline -----

    private static final AllianceSpecific<Measure<Distance>> STARTING_POS_Y = AllianceSpecific
        .forRed(SPEAKER_Y.in(Alliance.Red).plus(Units.Meters.of(1.04)))
        .forBlue(SPEAKER_Y.in(Alliance.Blue).minus(Units.Meters.of(1.04)));

    private static final AllianceSpecific<Rotation2d> STARTING_HEADING = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(60))
        .forBlue(Rotation2d.fromDegrees(-60));

    private static final AllianceSpecific<Pose2d> STARTING_POS = AllianceSpecific
        .forAlliance(alliance -> new Pose2d(
            Units.Meters.of(0.81),
            STARTING_POS_Y.in(alliance),
            STARTING_HEADING.in(alliance)));

    // ----- Strafe across centerline -----
    
    private static final AllianceSpecific<Translation2d> LOADSIDE_SHOOT_2 = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(4.5), Units.Meters.of(3.5)))
        .forBlue(new Translation2d(Units.Meters.of(4.5), Units.Meters.of(-3.5)));

    private static final AllianceSpecific<Translation2d> LOADSIDE_WAYPOINT_1 = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(4.2), Units.Meters.of(6.1)))
        .forBlue(new Translation2d(Units.Meters.of(4.2), Units.Meters.of(-6.1)));

    private static final AllianceSpecific<Translation2d> LOADSIDE_WAYPOINT_2 = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(6.9), Units.Meters.of(4.4)))
        .forBlue(new Translation2d(Units.Meters.of(7.4), Units.Meters.of(-5.4)));

    private static final AllianceSpecific<Translation2d> CENTER_3_OFFSET = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(0.1), Units.Meters.of(-0.4)))
        .forBlue(new Translation2d(Units.Meters.of(0.1), Units.Meters.of(0.2)));

    private static final AllianceSpecific<Translation2d> CENTER_4_OFFSET = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(0), Units.Meters.of(-0.1)))
        .forBlue(new Translation2d(Units.Meters.of(0), Units.Meters.of(0)));

    private static final AllianceSpecific<Translation2d> CENTER_5_OFFSET = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(0.2), Units.Meters.of(0)))
        .forBlue(new Translation2d(Units.Meters.of(0.2), Units.Meters.of(-0)));

    private static final AllianceSpecific<Rotation2d> SUBWOOFER_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(63))
        .forBlue(Rotation2d.fromDegrees(63));

    private static final AllianceSpecific<Rotation2d> SHOOT_ANGLE_1 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(29))
        .forBlue(Rotation2d.fromDegrees(29));

    private static final AllianceSpecific<Rotation2d> SHOOT_ANGLE_2 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(28.5))
        .forBlue(Rotation2d.fromDegrees(29));
    
    private static final AllianceSpecific<Rotation2d> SHOOT_ANGLE_3 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(28.5))
        .forBlue(Rotation2d.fromDegrees(29));

    // ----------------------------------------
    // Autonomous definition
    // ----------------------------------------
    public static Function<Scorpion, Actionable> createForAlliance(Alliance alliance) {
        return (robot) -> SEQUENCE(
            DO(() -> robot.drivetrain.setPosition(STARTING_POS.in(alliance))),
            DO (() -> robot.noteHandling.moveToAngle(SUBWOOFER_ANGLE.in(alliance))), //starting pos
            DO(robot.noteHandling::shooterSpinUp),
            WAIT(Units.Seconds.of(0.5)),
            robot.noteHandling.doShoot(), //shooting preload
            DO(robot.noteHandling::intake), //returning to intake mode
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_5.in(alliance).plus(CENTER_5_OFFSET.in(alliance)))  //going to center note
                .withWaypoints(LOADSIDE_WAYPOINT_1.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false)) 
                .precomputedWithStartPosition(STARTING_POS.in(alliance).getTranslation()),
            DO (() -> robot.noteHandling.moveToAngle(SHOOT_ANGLE_1.in(alliance))),
            robot.drivetrain
                .doMoveTo(LOADSIDE_SHOOT_2.in(alliance)) //shooting FIRST note
                .withWaypoints(LOADSIDE_WAYPOINT_2.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            WAIT(Units.Seconds.of(0.1)),
            DO(() -> robot.noteHandling.intake()),
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_4.in(alliance).plus(CENTER_4_OFFSET.in(alliance))) // going to second note
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            DO (() -> robot.noteHandling.moveToAngle(SHOOT_ANGLE_2.in(alliance))),
            robot.drivetrain
                .doMoveTo(LOADSIDE_SHOOT_2.in(alliance)) //shooting second note
                //.withWaypoints(LOADSIDE_WAYPOINT_3.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            WAIT(Units.Seconds.of(0.1)),
            DO(() -> robot.noteHandling.intake()),
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_3.in(alliance).plus(CENTER_3_OFFSET.in(alliance))) //third note
                //.withWaypoints(LOADSIDE_WAYPOINT_3.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))         
                .precomputedFromLastTarget(),
            DO (() -> robot.noteHandling.moveToAngle(SHOOT_ANGLE_3.in(alliance))),
            robot.drivetrain
                .doMoveTo(LOADSIDE_SHOOT_2.in(alliance)) //shooting the last
                //.withWaypoints(LOADSIDE_WAYPOINT_3.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            WAIT(Units.Seconds.of(0.1)),
            DO(() -> robot.noteHandling.intake())
           
        );
    }
}
