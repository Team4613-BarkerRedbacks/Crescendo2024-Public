package arachne4.lib.math.geometry;

import static arachne4.lib.math.ArachneMath.*;

import java.util.IdentityHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class FieldCoordinateSystem {
    public static final CoordinateConvention DEFAULT_ORIGIN = new CoordinateConvention("Default origin (in blue right corner)", (length, width) -> new Translation2d(), DEGREES_0);

    public static final CoordinateConvention
        BLUE_RIGHT_ORIGIN = DEFAULT_ORIGIN,
        BLUE_LEFT_ORIGIN = new CoordinateConvention("Origin in blue left corner", (length, width) -> new Translation2d(DISTANCE_0, width), DEGREES_0),
        RED_RIGHT_ORIGIN = new CoordinateConvention("Origin in red right corner", (length, width) -> new Translation2d(length, width), DEGREES_180),
        RED_LEFT_ORIGIN = new CoordinateConvention("Origin in red left corner", (length, width) -> new Translation2d(length, DISTANCE_0), DEGREES_180);

    public static final CoordinateConvention
        BLUE_WALL_CENTER_ORIGIN = new CoordinateConvention(
            "Origin in center of blue wall",
            (length, width) -> new Translation2d(DISTANCE_0, width.divide(2)),
            DEGREES_0),
        RED_WALL_CENTER_ORIGIN = new CoordinateConvention(
            "Origin in center of red wall",
            (length, width) -> new Translation2d(length, width.divide(2)),
            DEGREES_180);

    public static final CoordinateConvention
        FIELD_CENTER_ORIGIN_TOWARDS_RED = new CoordinateConvention(
            "Origin in center of field, positive X towards red wall",
            (length, width) -> new Translation2d(length.divide(2), width.divide(2)),
            DEGREES_0),
        FIELD_CENTER_ORIGIN_TOWARDS_BLUE = new CoordinateConvention(
            "Origin in center of field, positive X towards blue wall",
            (length, width) -> new Translation2d(length.divide(2), width.divide(2)),
            DEGREES_180);

    private final Measure<Distance> fieldLength, fieldWidth;
    private final IdentityHashMap<CoordinateConvention, Translation2d> originsCache;

    public FieldCoordinateSystem(Measure<Distance> fieldLength, Measure<Distance> fieldWidth) {
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;

        this.originsCache = new IdentityHashMap<>();
    }

    public Pose2d convert(Pose2d pose, CoordinateConvention from, CoordinateConvention to) {
        return new Pose2d(
            convert(pose.getTranslation(), from, to),
            convert(pose.getRotation(), from, to));
    }

    public Translation2d convert(Translation2d translation, CoordinateConvention from, CoordinateConvention to) {
        return translation
            .rotateBy(from.orientation)
            .plus(getCacheableOrigin(from))
            .minus(getCacheableOrigin(to))
            .rotateBy(to.orientation.unaryMinus());
    }

    public Rotation2d convert(Rotation2d rotation, CoordinateConvention from, CoordinateConvention to) {
        return to.convertRotationFrom(from, rotation);
    }

    public Pose3d convert(Pose3d pose, CoordinateConvention from, CoordinateConvention to) {
        return new Pose3d(
            convert(pose.getTranslation(), from, to),
            convert(pose.getRotation(), from, to));
    }

    public Translation3d convert(Translation3d translation, CoordinateConvention from, CoordinateConvention to) {
        Translation2d converted2d = convert(translation.toTranslation2d(), from, to);
        return new Translation3d(converted2d.getX(), converted2d.getY(), translation.getZ());
    }

    public Rotation3d convert(Rotation3d rotation, CoordinateConvention from, CoordinateConvention to) {
        return rotation.rotateBy(new Rotation3d(0, 0, from.orientation.minus(to.orientation).getRadians()));
    }

    private Translation2d getCacheableOrigin(CoordinateConvention convention) {
        Translation2d originInMap = originsCache.get(convention);
        if (originInMap != null) return originInMap;

        originsCache.put(convention, convention.originConversion.getFromFieldDimensions(fieldLength, fieldWidth));
        return originsCache.get(convention);
    }

    public static record CoordinateConvention(String name, ConversionFromBlueRightOrigin originConversion, Rotation2d orientation) {
        public Rotation2d convertRotationFrom(CoordinateConvention from, Rotation2d rotation) {
            return rotation
                .plus(from.orientation)
                .minus(orientation);
        }

        public Rotation2d convertRotationTo(CoordinateConvention to, Rotation2d rotation) {
            return rotation
                .plus(orientation)
                .minus(to.orientation);
        }
    }

    @FunctionalInterface
    public static interface ConversionFromBlueRightOrigin {
        Translation2d getFromFieldDimensions(Measure<Distance> fieldLength, Measure<Distance> fieldWidth);
    }
}
