package redbacks.lib.subsystems.positionbased;

public interface PositionBasedInputs<MeasureT> {
    MeasureT getCurrentPosition();
    MeasureT getTargetPosition();

    double convertToReadableUnits(MeasureT position);
    double convertToBaseUnits(MeasureT position);
}
