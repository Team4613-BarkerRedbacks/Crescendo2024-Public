package redbacks.robot.scorpion;

import static arachne4.lib.sequences.Actionable.*;
import static redbacks.field.FieldLocations.*;
import static redbacks.robot.subsystems.drivetrain.HeadingControlUtils.*;

import java.util.Map;
import java.util.function.Function;
import java.util.function.Predicate;

import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.math.Interpolators;
import arachne4.lib.sequences.Actionable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.robot.scorpion.subsystems.drivetrain.ScorpionDrivetrainConstants;
import redbacks.robot.scorpion.subsystems.noteHandling.ScorpionShooterConstants;

public class AutoChampsAmpSide {
    // ----------------------------------------
    // Constants
    // ----------------------------------------

    // ----- Initial move to centerline -----

    private static final AllianceSpecific<Measure<Distance>> STARTING_POS_Y = AllianceSpecific
        .forRed(EDGE_OF_AMP_ZONE_Y.in(Alliance.Red).plus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE))
        .forBlue(EDGE_OF_AMP_ZONE_Y.in(Alliance.Blue).minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE));

    private static final AllianceSpecific<Pose2d> STARTING_POS = AllianceSpecific
        .forAlliance(alliance -> new Pose2d(
            EDGE_OF_STARTING_ZONE_X.minus(ScorpionDrivetrainConstants.ROBOT_OFFSET_X_FROM_EDGE),
            STARTING_POS_Y.in(alliance),
            Rotation2d.fromDegrees(0)));

    private static final AllianceSpecific<Translation2d> WING_WAY_POINT = AllianceSpecific
        .forRed(new Translation2d(
            WING_NOTE_X.in(Units.Meters) - 0.1,
            WING_NOTE_1.in(Alliance.Red).getY() / 2 - 0.05))
        .forBlue(new Translation2d(
            WING_NOTE_X.in(Units.Meters) - 0.1,
            WING_NOTE_1.in(Alliance.Blue).getY() / 2 + 0.05));

    // ----- Strafe across centerline -----

    private static final AllianceSpecific<Translation2d> CENTRELINE_START_OFFSET = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(-0.1), Units.Meters.zero()))
        .forBlue(new Translation2d(Units.Meters.of(-0.1), Units.Meters.zero()));

    private static final AllianceSpecific<InterpolatingTreeMap<Double, Measure<Velocity<Distance>>>> DYNAMIC_SWEEP_SPEED = AllianceSpecific
        .forAlliance((alliance) -> Interpolators.interpolateOver(Interpolators.forDouble, Interpolators.forMeasure(),
            CENTER_NOTE_1.in(alliance).getY(), ScorpionShooterConstants.SWEEP_SPEED,
            CENTER_NOTE_2.in(alliance).getY(), ScorpionShooterConstants.SWEEP_SPEED.times(2)));

    private static final AllianceSpecific<Rotation2d> SWEEP_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(65))
        .forBlue(Rotation2d.fromDegrees(-65));

    private static final AllianceSpecific<Rotation2d> SWEEP_ANGLE_FOR_3RD_CENTER = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(45))
        .forBlue(Rotation2d.fromDegrees(-45));

    private static final AllianceSpecific<Predicate<Translation2d>> SWAP_FROM_SWEEP_TO_INTAKE_3 = AllianceSpecific.<Predicate<Translation2d>>
        forRed((pos) -> pos.getY() >= CENTER_NOTE_2.in(Alliance.Red).getY() + 1)
        .forBlue((pos) -> pos.getY() <= CENTER_NOTE_2.in(Alliance.Blue).getY() - 1);
 
    private static final AllianceSpecific<Predicate<Translation2d>> SWAP_FROM_SWEEP_TO_INTAKE_2 = AllianceSpecific.<Predicate<Translation2d>>
        forRed((pos) -> pos.getY() >= CENTER_NOTE_1.in(Alliance.Red).getY() + 0.2)
        .forBlue((pos) -> pos.getY() <= CENTER_NOTE_1.in(Alliance.Blue).getY() - 0.2);

    private static final AllianceSpecific<Translation2d> CENTRELINE_END_OFFSET = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(0.02), Units.Meters.of(0.15)))
        .forBlue(new Translation2d(Units.Meters.of(0.02), Units.Meters.of(-0.15)));

    // ----- Shoot last note on centerline -----

    private static final AllianceSpecific<Rotation2d> SHOT_ANGLE_1 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(28))
        .forBlue(Rotation2d.fromDegrees(28));

    private static final AllianceSpecific<Translation2d> MID_STAGE_SHOOT_POS = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.2, 3.8))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.2, -3.8));

    // ----- Recollect and shoot 2nd swept note -----

    private static final AllianceSpecific<Translation2d> BEHIND_STAGE_RECOLLECT_START_POS = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) + 0.8, MID_STAGE_SHOOT_POS.in(Alliance.Red).getY() + 0.2))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) + 0.8, MID_STAGE_SHOOT_POS.in(Alliance.Blue).getY() - 0.2));

    private static final AllianceSpecific<Rotation2d> BEHIND_STAGE_RECOLLECT_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(-90))
        .forBlue(Rotation2d.fromDegrees(90));

    private static final AllianceSpecific<Translation2d> BEHIND_STAGE_RECOLLECT_END_POS = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) + 0.8, 1.8))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) + 0.8, -1.8));

    private static final AllianceSpecific<Rotation2d> SHOT_ANGLE_2 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(28.5))
        .forBlue(Rotation2d.fromDegrees(28.5));

    private static final AllianceSpecific<Translation2d> CENTER_2_SHOOT_POS = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.4, 1.8))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.4, -1.8));

    // ----- Recollect and shoot 1st swept note -----

    private static final AllianceSpecific<Translation2d> CENTRE_1_PICKUP = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters) - 0.7, 0.7))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters) - 0.7, -1.4));

    private static final AllianceSpecific<Rotation2d> SHOT_ANGLE_3 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(28.2))
        .forBlue(Rotation2d.fromDegrees(28.6));

    private static final AllianceSpecific<Translation2d> CENTER_1_SHOOT_POS = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.4, 1.8))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.4, -1.8));

    private static final AllianceSpecific<Rotation2d> ROBOT_HEADING_OFFSET = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(0))
        .forBlue(Rotation2d.fromDegrees(4));

    // ----- Recollect and shoot preload -----

    private static final AllianceSpecific<Translation2d> PRELOAD_PICKUP = AllianceSpecific
        .forRed(new Translation2d(Units.Meters.of(5.2), Units.Meters.of(0.60)))
        .forBlue(new Translation2d(Units.Meters.of(5.2), Units.Meters.of(-0.60)));

    private static final AllianceSpecific<Rotation2d> SHOT_ANGLE_4 = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(28.5))
        .forBlue(Rotation2d.fromDegrees(28.5));

    private static final AllianceSpecific<Translation2d> PRELOAD_SHOOT_POS = AllianceSpecific
        .forRed(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.4, 1.8))
        .forBlue(new Translation2d(EDGE_OF_WING_X.in(Units.Meters) - 1.4, -1.8));

    private static final AllianceSpecific<Rotation2d> PRELOAD_PICKUP_ANGLE_START = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(-90))
        .forBlue(Rotation2d.fromDegrees(90));

    private static final AllianceSpecific<Rotation2d> PRELOAD_PICKUP_ANGLE_END = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(-20))
        .forBlue(Rotation2d.fromDegrees(20));


    // ----- Try for a last note -----

    private static final AllianceSpecific<Rotation2d> TRYING_ANGLE = AllianceSpecific
        .forRed(Rotation2d.fromDegrees(45))
        .forBlue(Rotation2d.fromDegrees(-45));

    private static final AllianceSpecific<Translation2d> BEGIN_TRY_SWEEP = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters), CENTER_NOTE_2.in(Alliance.Red).getY()))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters), CENTER_NOTE_2.in(Alliance.Blue).getY()));
        
    private static final AllianceSpecific<Translation2d> END_TRY_SWEEP = AllianceSpecific
        .forRed(new Translation2d(CENTER_NOTE_X.in(Units.Meters) + 0.1, CENTER_NOTE_5.in(Alliance.Red).getY()))
        .forBlue(new Translation2d(CENTER_NOTE_X.in(Units.Meters) + 0.1, CENTER_NOTE_5.in(Alliance.Blue).getY()));
        
    private static final AllianceSpecific<Translation2d> COMMON_AFTER_SWEEP_POINT = AllianceSpecific
        .forAlliance(alliance -> new Translation2d(EDGE_OF_WING_X.in(Units.Meters), CENTER_NOTE_3.in(alliance).getY()));

    // ----------------------------------------
    // Autonomous definition
    // ----------------------------------------
    public static Function<Scorpion, Actionable> createForAlliance(Alliance alliance) {
        return (robot) -> SEQUENCE(
            initalNote1AndEjectPreload(alliance).apply(robot),
        // NOTE 2 + 3
            DO(() -> robot.noteHandling.sweepWithDynamicSpeed(() -> DYNAMIC_SWEEP_SPEED.in(alliance).get(robot.drivetrain.getPosition().getY()), alliance.equals(Alliance.Red))),
            SPLIT(
                robot.drivetrain
                    .doMoveTo(CENTER_NOTE_3.in(alliance).plus(CENTRELINE_END_OFFSET.in(alliance)))
                    .withHeading(interpolateHeadingOnY(Map.of(
                        CENTER_NOTE_1.in(alliance), Rotation2d.fromDegrees(0),
                        CENTER_NOTE_2.in(alliance), SWEEP_ANGLE.in(alliance),
                        CENTER_NOTE_3.in(alliance), SWEEP_ANGLE_FOR_3RD_CENTER.in(alliance))))
                    .precomputedFromLastTarget()
            ).AND(
                SEQUENCE(
                    WAIT().UNSAFE_UNTIL(() -> SWAP_FROM_SWEEP_TO_INTAKE_3.in(alliance).test(robot.drivetrain.getPosition().getTranslation())),
                    DO(robot.noteHandling::intake)
                )
            ),
        // move to shot
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_1.in(alliance))),
            robot.drivetrain
                .doMoveTo(MID_STAGE_SHOOT_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
        // 2nd center note
            DO(robot.noteHandling::intake),
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_2.in(alliance))),
            robot.drivetrain
                .doMoveTo(BEHIND_STAGE_RECOLLECT_START_POS.in(alliance))
                .withHeading(interpolateHeadingOnX(Map.of(
                    MID_STAGE_SHOOT_POS.in(alliance), Rotation2d.fromDegrees(0),
                    BEHIND_STAGE_RECOLLECT_START_POS.in(alliance), BEHIND_STAGE_RECOLLECT_ANGLE.in(alliance))))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(BEHIND_STAGE_RECOLLECT_END_POS.in(alliance))
                .withHeading(BEHIND_STAGE_RECOLLECT_ANGLE.in(alliance))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(CENTER_2_SHOOT_POS.in(alliance))
                .withHeading(interpolateHeadingOnX(Map.of(
                    BEHIND_STAGE_RECOLLECT_END_POS.in(alliance), BEHIND_STAGE_RECOLLECT_ANGLE.in(alliance),
                    CENTER_2_SHOOT_POS.in(alliance), CENTER_2_SHOOT_POS.in(alliance).minus(SPEAKER.in(alliance)).getAngle())))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            centreNote1andPreload(alliance).apply(robot)
        );
    }

    public static Function<Scorpion, Actionable> createForAlliance2Notes(Alliance alliance) {
        return (robot) -> SEQUENCE(
            initalNote1AndEjectPreload(alliance).apply(robot),
        // NOTE 2 and shoot 
            DO(() -> robot.noteHandling.sweepWithDynamicSpeed(() -> DYNAMIC_SWEEP_SPEED.in(alliance).get(robot.drivetrain.getPosition().getY()), alliance.equals(Alliance.Red))),
            SPLIT(
                robot.drivetrain
                    .doMoveTo(CENTER_NOTE_2.in(alliance).plus(CENTRELINE_END_OFFSET.in(alliance)))
                    .withHeading(interpolateHeadingOnY(Map.of(
                        CENTER_NOTE_1.in(alliance), Rotation2d.fromDegrees(0),
                        CENTER_NOTE_2.in(alliance), SWEEP_ANGLE.in(alliance))))
                    .precomputedFromLastTarget()
            ).AND(
                SEQUENCE(
                    WAIT().UNSAFE_UNTIL(() -> SWAP_FROM_SWEEP_TO_INTAKE_2.in(alliance).test(robot.drivetrain.getPosition().getTranslation())),
                    DO(robot.noteHandling::intake)
                )
            ),
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_2.in(alliance))),
            robot.drivetrain
                .doMoveTo(BEHIND_STAGE_RECOLLECT_END_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(CENTER_2_SHOOT_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
            centreNote1andPreload(alliance).apply(robot),
        // Potential 3rd note 
            DO(robot.noteHandling::intake),
            robot.drivetrain
                .doMoveTo(BEHIND_STAGE_RECOLLECT_END_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(BEGIN_TRY_SWEEP.in(alliance))
                .withHeading(TRYING_ANGLE.in(alliance))
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(END_TRY_SWEEP.in(alliance))
                .withHeading(TRYING_ANGLE.in(alliance))
                .withMaxVelocity(Units.MetersPerSecond.of(4))
                .precomputedFromLastTarget()
                .UNSAFE_UNTIL(() -> robot.noteHandling.hasNote() == true),
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_1.in(alliance))),
            robot.drivetrain
                .doMoveTo(COMMON_AFTER_SWEEP_POINT.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false)),
            robot.drivetrain
                .doMoveTo(MID_STAGE_SHOOT_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot()
        );
    }

    private static Function<Scorpion, Actionable> initalNote1AndEjectPreload(Alliance alliance) {
    return (robot) -> SEQUENCE(
        DO(() -> robot.drivetrain.setPosition(STARTING_POS.in(alliance))),
        DO(robot.noteHandling::shooterSweepSpinUp),
    // NOTE 1
        SPLIT(
            robot.drivetrain
                .doMoveTo(CENTER_NOTE_1.in(alliance).plus(CENTRELINE_START_OFFSET.in(alliance)))
                .withWaypoints(WING_WAY_POINT.in(alliance))
                .withHeading(Rotation2d.fromDegrees(0))
                .precomputedWithStartPosition(STARTING_POS.in(alliance).getTranslation())
        ).AND(
            SEQUENCE(
                WAIT().UNSAFE_UNTIL(() -> robot.drivetrain.getPosition().getX() >= EDGE_OF_WING_X.in(Units.Meters) - 2.5),
                DO(robot.noteHandling::sweep))
            )
        );
    }

    private static Function<Scorpion, Actionable> centreNote1andPreload(Alliance alliance) {
    return (robot) -> SEQUENCE(
        // 1st center note
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_3.in(alliance))),
            DO(robot.noteHandling::intake),
            robot.drivetrain
                .doMoveTo(CENTRE_1_PICKUP.in(alliance))
                .withHeading(CENTRE_1_PICKUP.in(alliance).minus(CENTER_2_SHOOT_POS.in(alliance)).getAngle())
                .precomputedFromLastTarget(),
            robot.drivetrain
                .doMoveTo(CENTER_1_SHOOT_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false, ROBOT_HEADING_OFFSET.in(alliance)))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot(),
        // Preload
            DO(() -> robot.noteHandling.moveToAngle(SHOT_ANGLE_4.in(alliance))),
            DO(robot.noteHandling::intake),
            robot.drivetrain
                .doMoveTo(PRELOAD_PICKUP.in(alliance))
                .withWaypoints(PRELOAD_PICKUP.in(alliance).minus(new Translation2d(Units.Meters.of(1), Units.Meters.zero())))
                .withHeading(interpolateHeadingOnY(Map.of(
                    CENTER_1_SHOOT_POS.in(alliance), PRELOAD_PICKUP_ANGLE_START.in(alliance),
                    PRELOAD_PICKUP.in(alliance), PRELOAD_PICKUP_ANGLE_END.in(alliance))
                )
            )
            .precomputedFromLastTarget(),
            DO(robot.noteHandling::intakeWithShooterAtSpeed),
            robot.drivetrain
                .doMoveTo(PRELOAD_SHOOT_POS.in(alliance))
                .withHeading(aimAtPoint(SPEAKER.in(alliance), false))
                .precomputedFromLastTarget(),
            robot.noteHandling.doShoot()
        );
    }
}
