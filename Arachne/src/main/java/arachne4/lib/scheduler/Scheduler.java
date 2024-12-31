package arachne4.lib.scheduler;

import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import arachne4.lib.game.GameState;
import arachne4.lib.logging.ArachneLogger;
import arachne4.lib.scheduler.EventTypeSystem.BooleanDataEvent;
import arachne4.lib.scheduler.EventTypeSystem.DataEvent;
import arachne4.lib.scheduler.EventTypeSystem.DoubleDataEvent;
import arachne4.lib.scheduler.EventTypeSystem.Event;
import arachne4.lib.scheduler.data.ValueChange;
import arachne4.lib.scheduler.mappings.AxisMapping;
import arachne4.lib.scheduler.mappings.ButtonMapping;
import arachne4.lib.scheduler.mappings.ComplexMapping;
import arachne4.lib.scheduler.mappings.MappingManager;
import edu.wpi.first.util.function.BooleanConsumer;

public class Scheduler {
    private static final ArachneLogger log = ArachneLogger.getInstance();

    public static final Event
        INITIALIZE = new Event(),
        END = new Event();

    public static final DataEvent<GameState>
        PRE_EXECUTE = new DataEvent<>(),
        EXECUTE = new DataEvent<>(),
        POST_EXECUTE = new DataEvent<>();

    public static final DataEvent<ValueChange<GameState>> GAME_STATE_CHANGE = new DataEvent<>();

    public static final Event
        SIMULATION_INIT = new Event(),
        SIMULATION_EXECUTE = new Event();

    private final Map<Event, List<Runnable>> handlers = new HashMap<>();
    private final Map<DataEvent<?>, List<Consumer<?>>> dataHandlers = new HashMap<>();
    private final Map<BooleanDataEvent, List<BooleanConsumer>> booleanDataHandlers = new HashMap<>();
    private final Map<DoubleDataEvent, List<DoubleConsumer>> doubleDataHandlers = new HashMap<>();

    private final Map<Class<? extends MappingManager<?>>, MappingManager<?>> mappingsClassesMap = new HashMap<>();

    public void fire(Event event) {
        if(!handlers.containsKey(event)) return;

        for(var handler : handlers.get(event)) handler.run();
    }

    @SuppressWarnings("unchecked")
    public <DataT> void fire(DataEvent<DataT> event, DataT data) {
        if(!dataHandlers.containsKey(event)) return;

        for(var handler : dataHandlers.get(event)) ((Consumer<DataT>) handler).accept(data);
    }

    public void fire(BooleanDataEvent event, boolean data) {
        if(!booleanDataHandlers.containsKey(event)) return;

        for(var handler : booleanDataHandlers.get(event)) handler.accept(data);
    }

    public void fire(DoubleDataEvent event, double data) {
        if(!doubleDataHandlers.containsKey(event)) return;

        for(var handler : doubleDataHandlers.get(event)) handler.accept(data);
    }

    public void registerHandler(Event trigger, Runnable handler) {
        if(!handlers.containsKey(trigger)) handlers.put(trigger, new LinkedList<>());

        handlers.get(trigger).add(handler);
    }

    public <DataT> void registerHandler(DataEvent<DataT> trigger, Consumer<DataT> handler) {
        if(!dataHandlers.containsKey(trigger)) dataHandlers.put(trigger, new LinkedList<>());

        dataHandlers.get(trigger).add(handler);
    }

    public void registerHandler(BooleanDataEvent trigger, BooleanConsumer handler) {
        if(!booleanDataHandlers.containsKey(trigger)) booleanDataHandlers.put(trigger, new LinkedList<>());

        booleanDataHandlers.get(trigger).add(handler);
    }

    public void registerHandler(DoubleDataEvent trigger, DoubleConsumer handler) {
        if(!doubleDataHandlers.containsKey(trigger)) doubleDataHandlers.put(trigger, new LinkedList<>());

        doubleDataHandlers.get(trigger).add(handler);
    }

    @SuppressWarnings("unchecked")
    public void registerMappingsManager(MappingManager<?> mappingManager) {
        if (mappingManager.getClass().equals(MappingManager.class)) return;

        Class<? extends MappingManager<?>> mappingManagerClass = (Class<? extends MappingManager<?>>) mappingManager.getClass();

        if (mappingsClassesMap.containsKey(mappingManagerClass)) {
            log.critical(String.format("Duplicate mappings instance registered for %s, only 1 can be returned.", mappingManagerClass));
            return;
        }

        List<String> subOrSuperclasses = mappingsClassesMap.keySet().stream()
            .filter(key -> key.isAssignableFrom(mappingManagerClass) || mappingManagerClass.isAssignableFrom(key))
            .map(Class::getName)
            .toList();

        if (!subOrSuperclasses.isEmpty()) {
            log.error(String.format(
                "Registered mappings class %s is a sub/superclass of existing mappings classes: %s",
                mappingManagerClass.getName(),
                subOrSuperclasses));
            return;
        }

        mappingsClassesMap.put(mappingManagerClass, mappingManager);
    }

    @SuppressWarnings("unchecked")
    public <T extends MappingManager<?>> T getMappings(Class<T> mappingsClass) {
        if (mappingsClassesMap.containsKey(mappingsClass)) {
            log.info(String.format("Retrieved mappings instance for %s", mappingsClass.getSimpleName()));
            return (T) mappingsClassesMap.get(mappingsClass);
        }

        log.info(String.format("No direct mappings instance found for %s, searching for subclasses...", mappingsClass.getSimpleName()));

        List<Class<? extends MappingManager<?>>> possibleKeys = mappingsClassesMap.keySet().stream()
            .filter(key -> mappingsClass.isAssignableFrom(key))
            .sorted(Comparator.comparing(Class::getName))
            .toList();

        switch (possibleKeys.size()) {
            case 1:
                var key = possibleKeys.get(0);
                log.info(String.format("Found indirect mappings %s for key %s.", key.getSimpleName(), mappingsClass.getSimpleName()));
                return (T) mappingsClassesMap.get(key);
            case 0:
                log.critical(String.format(
                    "No mappings instance found for key %s, returning null. This will cause execution failures.",
                    mappingsClass.getName()));
                return null;
            default:
                log.critical(String.format(
                    "Multiple possible mappings instances found for key %s. Options are %s, the first of which (lexicographically) will be returned. This may cause execution failures.",
                    mappingsClass.getName(),
                    possibleKeys.stream().map(Class::getName)));
                return (T) mappingsClassesMap.get(possibleKeys.get(0));
        }
    }

    public ButtonMapping mapping(BooleanSupplier button) {
        return new ButtonMapping(this, button);
    }

    public AxisMapping mapping(DoubleSupplier axis) {
        return new AxisMapping(this, axis);
    }

    public <DataT> ComplexMapping<DataT> mapping(Supplier<DataT> dataProvider) {
        return new ComplexMapping<>(this, dataProvider);
    }
}
