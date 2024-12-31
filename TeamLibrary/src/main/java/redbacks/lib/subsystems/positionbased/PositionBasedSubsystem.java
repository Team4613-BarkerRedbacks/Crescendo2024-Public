package redbacks.lib.subsystems.positionbased;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import arachne4.lib.behaviours.Behaviour;
import arachne4.lib.behaviours.BehaviourBuilder;
import arachne4.lib.behaviours.BehaviourManager;
import arachne4.lib.logging.ArachneLogger;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.subsystems.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import redbacks.lib.subsystems.positionbased.PositionVisualiser.PositionVisualiserCreator;

public class PositionBasedSubsystem<
            IOT extends PositionBasedIO<InputsT, MeasureT>,
            InputsT extends PositionBasedInputs<MeasureT> & LoggableInputs,
            MeasureT
        > extends Subsystem {
    private final String name;
    private final PositionVisualiser<MeasureT> currentVisual, targetVisual;
    private final BehaviourManager<Behaviour> movementBehaviours;

    protected final IOT io;
    protected final InputsT inputs;

    private final Behaviour manuallyMoving = new Behaviour() {
        private double previousInput = Double.NaN;

        @Override
        public void run() {
            double input = manualMovementInput.getAsDouble();

            applyManualInputsWithLimitsCheck(input);
            whileManuallyMoving.accept(input);
        }

        private void applyManualInputsWithLimitsCheck(double input) {
            if (manualModePositionLimitBuffer.isPresent()) {
                double endpointBuffer = inputs.convertToBaseUnits(manualModePositionLimitBuffer.get());
                double currentPosition = inputs.convertToBaseUnits(inputs.getCurrentPosition());

                if (input >= 0 && currentPosition >= inputs.convertToBaseUnits(maximumPosition.get()) - endpointBuffer) setTargetPosition(maximumPosition.get());
                else if (input <= 0 && currentPosition <= inputs.convertToBaseUnits(minimumPosition.get()) + endpointBuffer) setTargetPosition(minimumPosition.get());
                else if (input != 0) io.setPercentageOutput(input);
                else if (previousInput != 0) setTargetPosition(inputs.getCurrentPosition());

                previousInput = input;
            }
            else {
                io.setPercentageOutput(input);
            }
        }
    };

    public PositionBasedSubsystem(String name, SchedulerProvider schedulerProvider, IOT io, MeasureT startPosition, PositionVisualiserCreator<MeasureT> visualiserCreator) {
        super(schedulerProvider);

        this.name = name;

        this.io = io;
        this.inputs = io.inputs();

        this.currentVisual = visualiserCreator.create("Actual", startPosition, Color.kDarkRed);
        this.targetVisual = visualiserCreator.create("Target", startPosition, Color.kGreen);

        io.setCurrentPosition(startPosition);

        this.movementBehaviours = new BehaviourManager<Behaviour>(schedulerProvider, manuallyMoving);
        registerHandler(Scheduler.EXECUTE, movementBehaviours);
    }

    /* Config */

    private Optional<MeasureT> minimumPosition = Optional.empty(), maximumPosition = Optional.empty();

    protected void configSetMinimumPosition(MeasureT minimumLimit) {
        this.minimumPosition = Optional.ofNullable(minimumLimit);
    }

    protected void configSetMaximumPosition(MeasureT maximumLimit) {
        this.maximumPosition = Optional.ofNullable(maximumLimit);
    }

    private DoubleSupplier manualMovementInput = () -> 0;
    private DoubleConsumer whileManuallyMoving = x -> {};
    private Optional<MeasureT> manualModePositionLimitBuffer = Optional.empty();

    protected void configSetManualInputSource(DoubleSupplier manualMovementInput) {
        this.manualMovementInput = manualMovementInput;
    }

    protected void configWhileManuallyMoving(DoubleConsumer whileManuallyMoving) {
        this.whileManuallyMoving = whileManuallyMoving;
    }

    protected void configSetApplyPositionLimitsInManualMode(MeasureT manualModePositionLimitBuffer) {
        this.manualModePositionLimitBuffer = Optional.ofNullable(manualModePositionLimitBuffer);
    }

    /* Operations */

    public void moveTo(MeasureT position) {
        movementBehaviours.changeToMode(Behaviour.thatRunsOnce(() -> setTargetPosition(position)));
    }

    protected void moveTo(Behaviour movementBehaviour) {
        movementBehaviours.changeToModeIfNotAlreadyIn(movementBehaviour);
    }

    protected void changeToManualMode() {
        movementBehaviours.changeToMode(manuallyMoving);
    }

    protected BehaviourBuilder createMoveToTargetBehaviour(MeasureT target) {
        return BehaviourBuilder.create().replaceOnEnterWith(() -> setTargetPosition(target));
    }

    protected BehaviourBuilder createDynamicFollowTargetBehaviour(Supplier<MeasureT> targetSupplier) {
        return BehaviourBuilder.create().replaceRunWith(() -> setTargetPosition(targetSupplier.get()));
    }

    private void setTargetPosition(MeasureT position) {
        if (minimumPosition.isPresent() && inputs.convertToBaseUnits(position) < inputs.convertToBaseUnits(minimumPosition.get())) {
            ArachneLogger.getInstance().error(String.format("Invalid %s target provided: %s", name, position));
            position = minimumPosition.get();
        }
        else if (maximumPosition.isPresent() && inputs.convertToBaseUnits(position) > inputs.convertToBaseUnits(maximumPosition.get())) {
            ArachneLogger.getInstance().error(String.format("Invalid %s target provided: %s", name, position));
            position = maximumPosition.get();
        }

        io.setTargetPosition(position);
    }

    { registerHandler(Scheduler.EXECUTE, this::checkManualInput); }
    private void checkManualInput() {
        if (manualMovementInput.getAsDouble() != 0) {
            movementBehaviours.changeToModeIfNotAlreadyIn(manuallyMoving);
            return;
        }
    }

    { registerHandler(Scheduler.POST_EXECUTE, this::telemetry); }
    private void telemetry() {
        SmartDashboard.putNumber(String.format("Current %s position", name), inputs.convertToReadableUnits(inputs.getCurrentPosition()));
        SmartDashboard.putNumber(String.format("Manual %s input", name), manualMovementInput.getAsDouble());

        currentVisual.update(inputs.getCurrentPosition());
        targetVisual.update(inputs.getTargetPosition() != null ? inputs.getTargetPosition() : inputs.getCurrentPosition());
    }
}
