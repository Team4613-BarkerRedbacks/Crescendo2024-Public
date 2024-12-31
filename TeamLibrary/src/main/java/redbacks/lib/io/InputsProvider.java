package redbacks.lib.io;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;

public abstract class InputsProvider<LoggableInputsT extends LoggableInputs> extends SchedulerProviderBase {
    public final LoggableInputsT inputs;

    public InputsProvider(Scheduler scheduler, String key, LoggableInputsT inputs) {
        super(scheduler);

        this.inputs = inputs;

        registerHandler(Scheduler.PRE_EXECUTE, () -> {
            updateInputs();
            Logger.processInputs(key, inputs);
        });
    }

    public LoggableInputsT inputs() {
        return inputs;
    }

    protected abstract void updateInputs();
}
