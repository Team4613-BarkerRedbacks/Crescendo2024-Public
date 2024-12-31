package arachne4.lib.behaviours;

import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.SchedulerProviderBase;
import arachne4.lib.scheduler.EventTypeSystem.DataEvent;
import arachne4.lib.sequences.Actionable;
import arachne4.lib.sequences.Untilable;
import arachne4.lib.sequences.actions.Action;

public class BehaviourManager<BehaviourT extends Behaviour> extends SchedulerProviderBase implements Runnable {
    protected Optional<BehaviourT> currentMode;
    protected Optional<ActionableBehaviour> currentActionMode;

    public DataEvent<BehaviourT>
        onEnter = new DataEvent<>(),
        run = new DataEvent<>(),
        onLeave = new DataEvent<>();

    protected final Supplier<BehaviourT> actionBackingBehaviourSupplier;

    public BehaviourManager(SchedulerProvider schedulerProvider, BehaviourT initialMode) {
        this(schedulerProvider, initialMode, () -> null);
    }

    public BehaviourManager(SchedulerProvider schedulerProvider, BehaviourT initialMode, BehaviourT actionBackingBehaviourInstance) {
        this(schedulerProvider, initialMode, () -> actionBackingBehaviourInstance);
    }

    public BehaviourManager(SchedulerProvider schedulerProvider, BehaviourT initialMode, Supplier<BehaviourT> actionBackingBehaviourSupplier) {
        super(schedulerProvider.getScheduler());

        currentMode = Optional.empty();
        currentActionMode = Optional.empty();

        changeToMode(initialMode);

        this.actionBackingBehaviourSupplier = actionBackingBehaviourSupplier;
    }

    public void run() {
        currentMode.ifPresent(Behaviour::run);
        currentActionMode.ifPresent(Behaviour::run);

        currentMode.ifPresent(mode -> getScheduler().fire(run, mode));
    }

    public void changeToMode(BehaviourT mode) {
        leaveCurrentMode();

        currentMode = Optional.ofNullable(mode);
        currentActionMode = Optional.empty();

        enterCurrentMode();
    }

    public void changeToModeIfNotAlreadyIn(BehaviourT mode) {
        if (currentMode.isEmpty() || !currentMode.get().equals(mode)) {
            changeToMode(mode);
        }
    }

    public Completable changeToMode(Actionable actionableAsMode) {
        leaveCurrentMode();

        currentMode = Optional.ofNullable(actionBackingBehaviourSupplier.get());

        var nonNullActionMode = new ActionableBehaviour(actionableAsMode);
        currentActionMode = Optional.of(nonNullActionMode);

        enterCurrentMode();

        return nonNullActionMode;
    }

    public Untilable doActionAsModeUntilComplete(Actionable actionable) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            Completable mode;

            @Override
            protected void initialize() {
                mode = changeToMode(actionable);
            }

            @Override
            protected boolean isFinished() {
                return mode.isComplete() || currentActionMode.orElse(null) != mode;
            }
        };
    }

    protected void enterCurrentMode() {
        currentMode.ifPresent(Behaviour::onEnterMode);
        currentActionMode.ifPresent(Behaviour::onEnterMode);

        currentMode.ifPresent(mode -> getScheduler().fire(onEnter, mode));
    }

    protected void leaveCurrentMode() {
        currentActionMode.ifPresent(Behaviour::onLeaveMode);
        currentMode.ifPresent(Behaviour::onLeaveMode);

        currentMode.ifPresent(mode -> getScheduler().fire(onLeave, mode));
    }

    public Optional<BehaviourT> getCurrentMode() {
        return currentMode;
    }

    public Optional<Completable> getCompletable() {
        return currentActionMode.map(Function.identity());
    }
}
