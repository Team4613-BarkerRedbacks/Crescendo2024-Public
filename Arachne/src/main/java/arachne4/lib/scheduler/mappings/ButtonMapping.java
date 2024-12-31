package arachne4.lib.scheduler.mappings;

import java.util.function.BooleanSupplier;

import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.EventTypeSystem.BooleanDataEvent;
import arachne4.lib.scheduler.EventTypeSystem.DataEvent;
import arachne4.lib.scheduler.EventTypeSystem.Event;
import arachne4.lib.scheduler.data.BooleanValueChange;

public class ButtonMapping {
    private Event onPressEvent, whilePressedEvent, onReleaseEvent;
    private BooleanDataEvent consumerEvent;
    private DataEvent<BooleanValueChange> onChangeEvent;

    private boolean isPressed = false;
    private boolean shouldReadInDisabled = false;

    public ButtonMapping(Scheduler scheduler, BooleanSupplier button) {
        scheduler.registerHandler(Scheduler.EXECUTE, (gameState) -> {
            // Entering auto moves the button state to false WITHOUT firing events
            if(gameState == GameState.AUTO) {
                isPressed = false;
                return;
            }

            boolean wasPressedLastCycle = isPressed;
            isPressed = gameState != GameState.DISABLED || shouldReadInDisabled ? button.getAsBoolean() : false;

            if(isPressed) {
                if(!wasPressedLastCycle && onPressEvent != null) scheduler.fire(onPressEvent);
                if(whilePressedEvent != null) scheduler.fire(whilePressedEvent);
            }
            else if(wasPressedLastCycle && onReleaseEvent != null) {
                scheduler.fire(onReleaseEvent);
            }

            if(isPressed != wasPressedLastCycle && onChangeEvent != null) scheduler.fire(onChangeEvent, new BooleanValueChange(wasPressedLastCycle, isPressed));

            if(consumerEvent != null) scheduler.fire(consumerEvent, isPressed);
        });
    }

    public ButtonMapping readEvenWhenDisabled() {
        shouldReadInDisabled = true;
        return this;
    }

    public Event onPress() {
        if(onPressEvent == null) onPressEvent = new Event();

        return onPressEvent;
    }

    public Event whilePressed() {
        if(whilePressedEvent == null) whilePressedEvent = new Event();

        return whilePressedEvent;
    }

    public Event onRelease() {
        if(onReleaseEvent == null) onReleaseEvent = new Event();

        return onReleaseEvent;
    }

    public BooleanDataEvent consume() {
        if(consumerEvent == null) consumerEvent = new BooleanDataEvent();

        return consumerEvent;
    }

    public DataEvent<BooleanValueChange> onChange() {
        if(onChangeEvent == null) onChangeEvent = new DataEvent<>();

        return onChangeEvent;
    }

    public boolean get() {
        return isPressed;
    }
}
