package redbacks.lib.subsystems.positionbased;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface PositionBasedIO<InputsT extends PositionBasedInputs<MeasureT> & LoggableInputs, MeasureT> {
    InputsT inputs();
    void updateInputs();
    void setPercentageOutput(double percent);
    void setCurrentPosition(MeasureT position);
    void setTargetPosition(MeasureT target);

    public static class Empty<InputsT extends PositionBasedInputs<MeasureT> & LoggableInputs, MeasureT> implements PositionBasedIO<InputsT, MeasureT> {
        public final InputsT inputs;

        public Empty(InputsT inputs) {
            this.inputs = inputs;
        }

        @Override
        public InputsT inputs() {
            return inputs;
        }

        @Override public void updateInputs() {}
        @Override public void setPercentageOutput(double percent) {}
        @Override public void setCurrentPosition(MeasureT position) {}
        @Override public void setTargetPosition(MeasureT target) {}
    }
}
