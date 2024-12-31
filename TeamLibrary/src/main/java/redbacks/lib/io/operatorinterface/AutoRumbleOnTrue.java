package redbacks.lib.io.operatorinterface;

import static arachne4.lib.sequences.Actionable.*;

import java.util.function.BooleanSupplier;

import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import arachne4.lib.sequences.ActionConductor;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class AutoRumbleOnTrue extends SchedulerProviderBase {
    boolean lastValue = false;

    public static void register(Scheduler scheduler, BooleanSupplier sensor, double rumbleStrength, Measure<Time> rumbleDuration, XboxController... controllers) {
        new AutoRumbleOnTrue(scheduler, sensor, rumbleStrength, rumbleDuration, controllers);
    }

    AutoRumbleOnTrue(Scheduler scheduler, BooleanSupplier sensor, double rumbleStrength, Measure<Time> rumbleDuration, XboxController... controllers) {
        super(scheduler);

        var conductor = new ActionConductor();

        var rumbleActionable = SEQUENCE(
            DO(() -> rumbleControllers(controllers, rumbleStrength)),
            WAIT(rumbleDuration),
            DO(() -> rumbleControllers(controllers, 0))
        );

        registerHandler(Scheduler.EXECUTE, (gameState) -> {
            var newValue = sensor.getAsBoolean();
            if(newValue && !lastValue && gameState != GameState.DISABLED) conductor.add(rumbleActionable);
            lastValue = newValue;

            conductor.run();
        });
    }

    static void rumbleControllers(XboxController[] controllers, double rumbleStrength) {
        for(XboxController controller : controllers) controller.setRumble(RumbleType.kLeftRumble, rumbleStrength);
    }
}
