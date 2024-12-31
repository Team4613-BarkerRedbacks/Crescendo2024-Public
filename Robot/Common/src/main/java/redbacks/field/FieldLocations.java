package redbacks.field;

import static arachne4.lib.math.geometry.FieldCoordinateSystem.*;

import java.util.HashMap;

import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.math.geometry.FieldCoordinateSystem;
import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldLocations {

    // ----------------------------------------
    // Field Dimensions
    // ----------------------------------------
    
    public static final Measure<Distance>  
        FIELD_LENGTH = Units.Feet.of(54).plus(Units.Inches.of(3.25)),
        FIELD_WIDTH = Units.Feet.of(26).plus(Units.Inches.of(11.25));

    public static final FieldCoordinateSystem FIELD_COORDINATE_SYSTEM = new FieldCoordinateSystem(FIELD_LENGTH, FIELD_WIDTH);

    public static final AllianceSpecific<CoordinateConvention> COORDINATE_CONVENTION = AllianceSpecific
        .forRed(RED_RIGHT_ORIGIN)
        .forBlue(BLUE_LEFT_ORIGIN);

    // ----- Zone boundaries -----
    
    public static final Measure<Distance>
        EDGE_OF_STARTING_ZONE_X = Units.Feet.of(6).plus(Units.Inches.of(4.125)),
        EDGE_OF_WING_X = Units.Meters.of(5.87);

    public static final AllianceSpecific<Measure<Distance>> EDGE_OF_AMP_ZONE_Y = createAllianceSpecificFromRedY(Units.Meters.of(0.46));

    // ----- Speaker and subwoofer -----
    
    public static final AllianceSpecific<Measure<Distance>> SPEAKER_Y = createAllianceSpecificFromRedY(Units.Feet.of(8).plus(Units.Inches.of(8.625)));
    public static final AllianceSpecific<Translation2d> SPEAKER = AllianceSpecific.forAlliance(alliance -> new Translation2d(Units.Meters.zero(), SPEAKER_Y.in(alliance)));

    public static final Measure<Distance> EDGE_OF_SUBWOOFER_X = Units.Feet.of(3).plus(Units.Inches.of(0.125));

    // ----------------------------------------
    // Note Locations
    // ----------------------------------------

    // ----- Wing notes -----

    public static final Measure<Distance>
        WING_NOTE_X = Units.Feet.of(9).plus(Units.Inches.of(6)),
        WING_NOTE_Y_SEPARATION = Units.Feet.of(4).plus(Units.Inches.of(9));

    private static final AllianceSpecific<Measure<Distance>>
        WING_NOTE_3_Y = createAllianceSpecificFromRedY(FIELD_WIDTH.divide(2)),
        WING_NOTE_2_Y = createAllianceSpecificFromRedY(WING_NOTE_3_Y.in(Alliance.Red).minus(WING_NOTE_Y_SEPARATION)),
        WING_NOTE_1_Y = createAllianceSpecificFromRedY(WING_NOTE_2_Y.in(Alliance.Red).minus(WING_NOTE_Y_SEPARATION));

    public static final AllianceSpecific<Translation2d>
        WING_NOTE_1 = AllianceSpecific.forAlliance(alliance -> new Translation2d(WING_NOTE_X, WING_NOTE_1_Y.in(alliance))),
        WING_NOTE_2 = AllianceSpecific.forAlliance(alliance -> new Translation2d(WING_NOTE_X, WING_NOTE_2_Y.in(alliance))),
        WING_NOTE_3 = AllianceSpecific.forAlliance(alliance -> new Translation2d(WING_NOTE_X, WING_NOTE_3_Y.in(alliance)));

    // ----- Center notes -----

    public static final Measure<Distance>
        CENTER_NOTE_X = FIELD_LENGTH.divide(2),
        CENTER_NOTE_SEPARATION = Units.Feet.of(5).plus(Units.Inches.of(6));

    private static final AllianceSpecific<Measure<Distance>> 
        CENTER_NOTE_3_Y = createAllianceSpecificFromRedY(FIELD_WIDTH.divide(2)),
        CENTER_NOTE_2_Y = createAllianceSpecificFromRedY(CENTER_NOTE_3_Y.in(Alliance.Red).minus(CENTER_NOTE_SEPARATION)),
        CENTER_NOTE_1_Y = createAllianceSpecificFromRedY(CENTER_NOTE_2_Y.in(Alliance.Red).minus(CENTER_NOTE_SEPARATION)),
        CENTER_NOTE_4_Y = createAllianceSpecificFromRedY(CENTER_NOTE_3_Y.in(Alliance.Red).plus(CENTER_NOTE_SEPARATION)),
        CENTER_NOTE_5_Y = createAllianceSpecificFromRedY(CENTER_NOTE_4_Y.in(Alliance.Red).plus(CENTER_NOTE_SEPARATION));

    public static final AllianceSpecific<Translation2d>
        CENTER_NOTE_1 = AllianceSpecific.forAlliance(alliance -> new Translation2d(CENTER_NOTE_X, CENTER_NOTE_1_Y.in(alliance))),
        CENTER_NOTE_2 = AllianceSpecific.forAlliance(alliance -> new Translation2d(CENTER_NOTE_X, CENTER_NOTE_2_Y.in(alliance))),
        CENTER_NOTE_3 = AllianceSpecific.forAlliance(alliance -> new Translation2d(CENTER_NOTE_X, CENTER_NOTE_3_Y.in(alliance))),
        CENTER_NOTE_4 = AllianceSpecific.forAlliance(alliance -> new Translation2d(CENTER_NOTE_X, CENTER_NOTE_4_Y.in(alliance))),
        CENTER_NOTE_5 = AllianceSpecific.forAlliance(alliance -> new Translation2d(CENTER_NOTE_X, CENTER_NOTE_5_Y.in(alliance)));

    // ----------------------------------------
    // AprilTags
    // ----------------------------------------

    // Real field locations
    public static HashMap<Integer, AllianceSpecific<Pose3d>> APRIL_TAGS = new HashMap<>()
    {{
        put(1, getAprilTagPosFromWpiCoords(Units.Inches.of(593.68), Units.Inches.of(9.68), Units.Inches.of(53.38), Rotation2d.fromDegrees(120)));
        put(2, getAprilTagPosFromWpiCoords(Units.Inches.of(637.21), Units.Inches.of(34.79), Units.Inches.of(53.38), Rotation2d.fromDegrees(120)));
        put(3, getAprilTagPosFromWpiCoords(Units.Inches.of(652.73), Units.Inches.of(196.17), Units.Inches.of(57.13), Rotation2d.fromDegrees(180)));
        put(4, getAprilTagPosFromWpiCoords(Units.Inches.of(652.73), Units.Inches.of(218.42), Units.Inches.of(57.13), Rotation2d.fromDegrees(180)));
        put(5, getAprilTagPosFromWpiCoords(Units.Inches.of(578.77), Units.Inches.of(323.00), Units.Inches.of(53.38), Rotation2d.fromDegrees(270)));
        put(6, getAprilTagPosFromWpiCoords(Units.Inches.of(72.5), Units.Inches.of(323.00), Units.Inches.of(53.38), Rotation2d.fromDegrees(270)));
        put(7, getAprilTagPosFromWpiCoords(Units.Inches.of(-1.5), Units.Inches.of(218.42), Units.Inches.of(57.13), Rotation2d.fromDegrees(0)));
        put(8, getAprilTagPosFromWpiCoords(Units.Inches.of(-1.5), Units.Inches.of(196.17), Units.Inches.of(57.13), Rotation2d.fromDegrees(0)));
        put(9, getAprilTagPosFromWpiCoords(Units.Inches.of(14.02), Units.Inches.of(34.79), Units.Inches.of(53.38), Rotation2d.fromDegrees(60)));
        put(10, getAprilTagPosFromWpiCoords(Units.Inches.of(57.54), Units.Inches.of(9.68), Units.Inches.of(53.38), Rotation2d.fromDegrees(60)));
        put(11, getAprilTagPosFromWpiCoords(Units.Inches.of(468.69), Units.Inches.of(146.19), Units.Inches.of(52), Rotation2d.fromDegrees(300)));
        put(12, getAprilTagPosFromWpiCoords(Units.Inches.of(468.69), Units.Inches.of(177.1), Units.Inches.of(52), Rotation2d.fromDegrees(60)));
        put(13, getAprilTagPosFromWpiCoords(Units.Inches.of(441.4), Units.Inches.of(161.62), Units.Inches.of(52), Rotation2d.fromDegrees(180)));
        put(14, getAprilTagPosFromWpiCoords(Units.Inches.of(209.48), Units.Inches.of(161.62), Units.Inches.of(52), Rotation2d.fromDegrees(0)));
        put(15, getAprilTagPosFromWpiCoords(Units.Inches.of(182.73), Units.Inches.of(177.1), Units.Inches.of(52), Rotation2d.fromDegrees(120)));
        put(16, getAprilTagPosFromWpiCoords(Units.Inches.of(182.73), Units.Inches.of(146.19), Units.Inches.of(52), Rotation2d.fromDegrees(240)));
    }};

    private static AllianceSpecific<Pose3d> getAprilTagPosFromWpiCoords(Measure<Distance> x, Measure<Distance> y, Measure<Distance> z, Rotation2d angle) {
        return AllianceSpecific
            .forAlliance(alliance -> new Pose3d(
                FIELD_COORDINATE_SYSTEM.convert(new Translation3d(x, y, z), DEFAULT_ORIGIN, COORDINATE_CONVENTION.in(alliance)),
                new Rotation3d(0, 0, FIELD_COORDINATE_SYSTEM.convert(angle, DEFAULT_ORIGIN, COORDINATE_CONVENTION.in(alliance)).getRadians())));
    }

    // ----------------------------------------
    // Private Helpers
    // ----------------------------------------

    private static AllianceSpecific<Measure<Distance>> createAllianceSpecificFromRedY(Measure<Distance> fieldLocation) {
        return AllianceSpecific
            .forRed(fieldLocation)
            .forBlue(fieldLocation.negate());
    }
}