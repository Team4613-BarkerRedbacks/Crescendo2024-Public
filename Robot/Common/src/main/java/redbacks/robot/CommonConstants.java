package redbacks.robot;

import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.math.Interpolators;
import arachne4.lib.math.geometry.FieldCoordinateSystem;
import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.field.FieldLocations;

public class CommonConstants {
    public static final String CANIVORE_BUS_NAME = "canivore";

	public static final int
        CAN_TIMEOUT = 0,
        CANCODER_TIMEOUT = 100,
        CANCODER_STATUS_FRAME_PERIOD = 250,
        REINIT_ATTEMPT_LIMIT = 5;

    public static final Measure<Time>
        LOOP_PERIOD = Units.Seconds.of(0.01),
        REINIT_DELAY = Units.Seconds.of(1);

    public static final AllianceSpecific<CoordinateConvention> COORDINATE_CONVENTIONS = AllianceSpecific
        .forRed(FieldCoordinateSystem.RED_RIGHT_ORIGIN)
        .forBlue(FieldCoordinateSystem.BLUE_LEFT_ORIGIN);

    public static final AllianceSpecific<Translation2d> FEED_TO_AMP_TARGET_POS = AllianceSpecific
        .forRed(new Translation2d(
            FieldLocations.EDGE_OF_STARTING_ZONE_X.in(Units.Meters),
            FieldLocations.WING_NOTE_1.in(Alliance.Red).getY()))
        .forBlue(new Translation2d(
            FieldLocations.EDGE_OF_STARTING_ZONE_X.in(Units.Meters),
            FieldLocations.WING_NOTE_1.in(Alliance.Blue).getY()));

    public static final AllianceSpecific<Translation2d> FEED_TO_CENTER_TARGET_POS = AllianceSpecific
        .forRed(new Translation2d(
            FieldLocations.CENTER_NOTE_X.in(Units.Meters),
            FieldLocations.CENTER_NOTE_1.in(Alliance.Red).getY() + 0.5))
        .forBlue(new Translation2d(
            FieldLocations.CENTER_NOTE_X.in(Units.Meters),
            FieldLocations.CENTER_NOTE_1.in(Alliance.Blue).getY() - 0.5));

    public static final InterpolatingTreeMap<Double, Rotation2d> FEED_TO_CENTER_YAW_OFFSET_FROM_Y_VELOCITY =
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forRotation2d)
        {{
            put(0.0, Rotation2d.fromDegrees(0));
            put(3.0, Rotation2d.fromDegrees(30));
        }};

    public static final InterpolatingTreeMap<Double, Rotation2d> FEED_TO_AMP_YAW_OFFSET_FROM_Y_VELOCITY =
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forRotation2d)
        {{
            put(0.0, Rotation2d.fromDegrees(0));
            put(3.0, Rotation2d.fromDegrees(30));
        }};
}
