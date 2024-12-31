package arachne4.lib.math.geometry;

import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;
import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dWithCoordinateConvention extends ValueWithCoordinateConvention<Rotation2d> {
    public Rotation2dWithCoordinateConvention(CoordinateConvention coordinateConvention, Rotation2d angle) {
        super(coordinateConvention, angle);
    }

    @Override
    public Rotation2d getIn(CoordinateConvention conventionToConvertTo) {
        return conventionToConvertTo.convertRotationFrom(this.coordinateConvention, this.value);
    }
}
