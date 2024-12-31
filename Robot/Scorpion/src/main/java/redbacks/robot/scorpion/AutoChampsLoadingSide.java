package redbacks.robot.scorpion;

import static arachne4.lib.sequences.Actionable.*;
import static redbacks.field.FieldLocations.*;
import static redbacks.robot.subsystems.drivetrain.HeadingControlUtils.*;

import java.util.Map;
import java.util.function.Function;
import java.util.function.Predicate;

import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.sequences.Actionable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.scorpion.subsystems.drivetrain.ScorpionDrivetrainConstants;

public class AutoChampsLoadingSide {

    // ----------------------------------------
    // Constants
    // ----------------------------------------

    // ----- Initial move to centreline -----

    private static final AllianceSpecific<Measure<Distance>> STARTING_POS_Y = AllianceSpecific
        .forRed(FIELD_WIDTH.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE).minus(Units.Meters.of(0.87)))
        .forBlue(FIELD_WIDTH.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_Y_FROM_EDGE).minus(Units.Meters.of(0.87)).negate());

    private static final AllianceSpecific<Pose2d> STARTING_POS = AllianceSpecific
        .forRed(new Pose2d(
            EDGE_OF_STARTING_ZONE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE),
            STARTING_POS_Y.in(Alliance.Red),
            Rotation2d.fromDegrees(0)))
        .forBlue(new Pose2d(
            EDGE_OF_STARTING_ZONE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE),
            STARTING_POS_Y.in(Alliance.Blue),
            Rotation2d.fromDegrees(0)));

    // ----- Sweep along the centreline -----

    private static final AllianceSpecific<Rotation2d> SWEEP_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(-80))
        .forBlue(Rotation2d.fromDegrees(80));

    private static final AllianceSpecific<Predicate<Translation2d>> START_INTAKING = AllianceSpecific.<Predicate<Translation2d>>
        forRed((pos) -> pos.getY() <= CENTER_NOTE_5.in(Alliance.Red).getY() - 0.3)
        .forBlue((pos) -> pos.getY() >= CENTER_NOTE_5.in(Alliance.Blue).getY() + 0.3);
    
    // ----- Move to shoot 4th centreline note -----

    private static final AllianceSpecific<Translation2d> STAGE_WAY_POINT = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1, FIELD_WIDTH.in(Units.Meters) / 2 + 2.5))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1, -FIELD_WIDTH.in(Units.Meters) / 2 - 2.5));
    
    private static final AllianceSpecific<Translation2d> SHOOT_POSITION_1 = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 2, FIELD_WIDTH.in(Units.Meters) / 2 + 1))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 2, -FIELD_WIDTH.in(Units.Meters) / 2 - 1));

    // ----- Move to intake 5th centreline note -----  

    private static final AllianceSpecific<Translation2d> PRE_PICKUP_NOTE_5 = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters) - 1.5, CENTER_NOTE_5.in(Alliance.Red).getY() - 0))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters) - 2.5, CENTER_NOTE_5.in(Alliance.Blue).getY() + 0.7));
    
    private static final AllianceSpecific<Translation2d> PICKUP_NOTE_5 = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters) - 0.8, CENTER_NOTE_5.in(Alliance.Red).getY() - 0))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters) - 1, CENTER_NOTE_5.in(Alliance.Blue).getY() + 0.7));
        
    // ----- Move to shoot 5th centreline note -----  
    
    private static final AllianceSpecific<Translation2d> SHOOT_POSITION_2 = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 2, FIELD_WIDTH.in(Units.Meters) / 2 + 1))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 2, -FIELD_WIDTH.in(Units.Meters) / 2 - 1));
     
    // ----- Move to intake and shoot the preload -----

    private static final AllianceSpecific<Translation2d> SHOOT_POSITION_3 = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 2, FIELD_WIDTH.in(Units.Meters) / 2 + 1))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 2, -FIELD_WIDTH.in(Units.Meters) / 2 - 1));
    
    private static final AllianceSpecific<Translation2d> PICKUP_PRELOAD = AllianceSpecific
        .forRed(new Translation2d(4.6, FIELD_WIDTH.in(Units.Meters) - 0.87))
        .forBlue(new Translation2d(4.1, -FIELD_WIDTH.in(Units.Meters) + 0.87));

    // ----- Pivot angle -----

    private static final Rotation2d SHOT_ANGLE_1 = Rotation2d.fromDegrees(28);
    private static final Rotation2d SHOT_ANGLE_2 = Rotation2d.fromDegrees(28.5);
    private static final Rotation2d SHOT_ANGLE_3 = Rotation2d.fromDegrees(28);

    // ----- Yaw offsets from directly at the goal for shooting -----

    private static final AllianceSpecific<Rotation2d> SHOT_1_YAW_OFFSET = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(4))
        .forBlue(Rotation2d.fromDegrees(-4));

    private static final AllianceSpecific<Rotation2d> SHOT_2_YAW_OFFSET = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(4))
        .forBlue(Rotation2d.fromDegrees(-4));

    private static final AllianceSpecific<Rotation2d> SHOT_3_YAW_OFFSET = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(4))
        .forBlue(Rotation2d.fromDegrees(-4));

    // ----------------------------------------
    // Autonomous definition
    // ----------------------------------------

    public static Function<Scorpion, Actionable> createForAlliance(Alliance alliance) {
        return (robot) -> SEQUENCE(
            DO(() -> robot.drivetrain.setPosition(STARTING_POS.in(alliance))),
            DO(robot.noteHandling::shooterSweepSpinUp),
        // Inital move to centerline 
            SPLIT(
                robot.drivetrain.doMoveTo(CENTER_NOTE_5.in(alliance).plus(new Translation2d(0.1,0)))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedWithStartPosition(STARTING_POS.in(alliance).getTranslation())
            ).AND(
                SEQUENCE(
                    WAIT().UNSAFE_UNTIL(() -> robot.drivetrain.getPosition().getX() >= EDGE_OF_WING_X.in(Units.Meters) - 3),
                    DO(robot.noteHandling::sweep)
                )
            ),
        // Sweep the 5th note and intake the 4th note
            SPLIT(
                robot.drivetrain.doMoveTo(CENTER_NOTE_4.in(alliance).plus(new Translation2d(0.1,0)))
                .withHeading(interpolateHeadingOnY(Map.of(
                    CENTER_NOTE_5.in(alliance), Rotation2d.fromDegrees(0),
                    CENTER_NOTE_4.in(alliance), SWEEP_ANGLE.in(alliance))))
                .precomputedFromLastTarget()
            ).AND(
                SEQUENCE(
                    WAIT().UNSAFE_UNTIL(() -> START_INTAKING.in(alliance).test(robot.drivetrain.getPosition().getTranslation())),
                    DO(robot.noteHandling::intake)
                )
            ),
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_1)),
        //Move to shoot 4th centreline note
            robot.drivetrain.doMoveTo(
                STAGE_WAY_POINT.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),
            robot.drivetrain.doMoveTo(
                SHOOT_POSITION_1.in(alliance))
                .withHeading((aimAtPoint(SPEAKER.in(alliance), false, SHOT_1_YAW_OFFSET.in(alliance))))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
        //Move to intake and shoot the swept 5th centreline note
            DO(robot.noteHandling::intake),
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_2)),
            robot.drivetrain.doMoveTo(
                PRE_PICKUP_NOTE_5.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),
            robot.drivetrain.doMoveTo(
                PICKUP_NOTE_5.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),
            robot.drivetrain.doMoveTo(
                STAGE_WAY_POINT.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedFromLastTarget(),
            robot.drivetrain.doMoveTo(
                SHOOT_POSITION_2.in(alliance))
                .withHeading((aimAtPoint(SPEAKER.in(alliance), false, SHOT_2_YAW_OFFSET.in(alliance))))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
        //Move to intake and shoot the preload
            DO(robot.noteHandling::intake),
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_3)),
            robot.drivetrain
                .doMoveTo(PICKUP_PRELOAD.in(alliance))
                .withHeading(PICKUP_PRELOAD.in(alliance).minus(SHOOT_POSITION_3.in(alliance)).getAngle())
                .precomputedFromLastTarget(),
            WAIT(Units.Seconds.of(0.3)),
            robot.drivetrain.doMoveTo(
                SHOOT_POSITION_3.in(alliance))
                .withHeading((aimAtPoint(SPEAKER.in(alliance), false, SHOT_3_YAW_OFFSET.in(alliance))))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot()
        );
    } 
}
