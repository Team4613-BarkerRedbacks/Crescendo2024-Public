package arachne4.lib.scheduler.mappings;

import java.util.Objects;
import java.util.function.Supplier;

import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.EventTypeSystem.DataEvent;
import arachne4.lib.scheduler.data.ValueChange;

public class ComplexMapping<DataT> {
    private DataEvent<DataT> consumerEvent;
    private DataEvent<ValueChange<DataT>> onChangeEvent;

    private DataT value = null;
    private boolean shouldReadInDisabled = false;

    public ComplexMapping(Scheduler scheduler, Supplier<DataT> dataSupplier) {
        scheduler.registerHandler(Scheduler.EXECUTE, (gameState) -> {
            // Entering auto moves the mapping state to null WITHOUT firing events
            if(gameState == GameState.AUTO) {
                value = null;
                return;
            }

            DataT valueLastCycle = value;
            value = gameState != GameState.DISABLED || shouldReadInDisabled ? dataSupplier.get() : null;

            if(!Objects.equals(value, valueLastCycle) && onChangeEvent != null) scheduler.fire(onChangeEvent, new ValueChange<DataT>(valueLastCycle, value));

            if(consumerEvent != null) scheduler.fire(consumerEvent, value);
        });
    }

    public ComplexMapping<DataT> readEvenWhenDisabled() {
        shouldReadInDisabled = true;
        return this;
    }

    public DataEvent<DataT> consume() {
        if(consumerEvent == null) consumerEvent = new DataEvent<>();

        return consumerEvent;
    }

    public DataEvent<ValueChange<DataT>> onChange() {
        if(onChangeEvent == null) onChangeEvent = new DataEvent<>();

        return onChangeEvent;
    }

    public DataT get() {
        return value;
    }
}
