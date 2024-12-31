package arachne4.lib.scheduler.data;

import java.util.Objects;

import arachne4.lib.scheduler.EventTypeSystem.DataEvent;
import arachne4.lib.scheduler.Scheduler;

public class DataSource<DataT> {
    private final Scheduler scheduler;

    private DataEvent<DataT> consumerEvent;
    private DataEvent<ValueChange<DataT>> onChangeEvent;

    private DataT value;

    public DataSource(Scheduler scheduler, DataT initialValue) {
        this.scheduler = scheduler;
        this.value = initialValue;

        scheduler.registerHandler(Scheduler.EXECUTE, (gameState) -> {
            if (consumerEvent != null) scheduler.fire(consumerEvent, value);
        });
    }

    public DataT get() {
        return value;
    }

    public void set(DataT newValue) {
        if(Objects.equals(value, newValue)) return;

        if (onChangeEvent != null) scheduler.fire(onChangeEvent, new ValueChange<DataT>(value, newValue));
        value = newValue;
    }

    public DataEvent<DataT> consume() {
        if(consumerEvent == null) consumerEvent = new DataEvent<>();
        return consumerEvent;
    }

    public DataEvent<ValueChange<DataT>> onChange() {
        if(onChangeEvent == null) onChangeEvent = new DataEvent<>();
        return onChangeEvent;
    }
}
