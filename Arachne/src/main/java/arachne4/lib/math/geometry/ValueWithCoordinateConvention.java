package arachne4.lib.math.geometry;

import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;

public abstract class ValueWithCoordinateConvention<ValueT> {
    protected final CoordinateConvention coordinateConvention;
    protected final ValueT value;

    public ValueWithCoordinateConvention(CoordinateConvention coordinateConvention, ValueT value) {
        this.coordinateConvention = coordinateConvention;
        this.value = value;
    }

    public abstract ValueT getIn(CoordinateConvention conventionToConvertTo);
}
