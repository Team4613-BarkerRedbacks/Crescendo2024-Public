package redbacks.lib.subsystems.positionbased;

import edu.wpi.first.wpilibj.util.Color;

public interface PositionVisualiser<MeasureT> {
    void update(MeasureT position);

    @FunctionalInterface
    public static interface PositionVisualiserCreator<MeasureT> {
        PositionVisualiser<MeasureT> create(String name, MeasureT initialPosition, Color color);

        public static <MeasureT> PositionVisualiserCreator<MeasureT> none() {
            return (name, initialPosition, color) -> position -> {};
        }
    }
}
