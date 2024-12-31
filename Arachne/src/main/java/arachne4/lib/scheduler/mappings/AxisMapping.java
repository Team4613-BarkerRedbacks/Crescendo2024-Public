package arachne4.lib.scheduler.mappings;

import java.util.function.DoubleSupplier;

import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.EventTypeSystem.DataEvent;
import arachne4.lib.scheduler.EventTypeSystem.DoubleDataEvent;
import arachne4.lib.scheduler.data.DoubleValueChange;

public class AxisMapping {
    private DoubleDataEvent consumerEvent;
    private DataEvent<DoubleValueChange> onChangeEvent;

    private double value = 0;
    private boolean shouldReadInDisabled = false;

    public AxisMapping(Scheduler scheduler, DoubleSupplier axis) {
        scheduler.registerHandler(Scheduler.EXECUTE, (gameState) -> {
            // Entering auto moves the axis state to 0 WITHOUT firing events
            if(gameState == GameState.AUTO) {
                value = 0;
                return;
            }

            double valueLastCycle = value;
            value = gameState != GameState.DISABLED || shouldReadInDisabled ? axis.getAsDouble() : 0;

            if(value != valueLastCycle && onChangeEvent != null) scheduler.fire(onChangeEvent, new DoubleValueChange(valueLastCycle, value));

            if(consumerEvent != null) scheduler.fire(consumerEvent, value);
        });
    }

    public AxisMapping readEvenWhenDisabled() {
        shouldReadInDisabled = true;
        return this;
    }

    public DoubleDataEvent consume() {
        if(consumerEvent == null) consumerEvent = new DoubleDataEvent();

        return consumerEvent;
    }

    public DataEvent<DoubleValueChange> onChange() {
        if(onChangeEvent == null) onChangeEvent = new DataEvent<>();

        return onChangeEvent;
    }

    public double get() {
        return value;
    }
}
