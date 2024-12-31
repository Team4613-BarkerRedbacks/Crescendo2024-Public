package redbacks.lib.subsystems.positionbased;

import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import redbacks.lib.hardware.motor.SmartWheeledMotor;
import redbacks.lib.io.InputsProvider;

public class DistanceBasedMotorHardware extends InputsProvider<DistanceBasedInputsAutoLogged> implements DistanceBasedIO {
    protected final SmartWheeledMotor positionalMotor;

    public DistanceBasedMotorHardware(SchedulerProvider schedulerProvider, String key, SmartWheeledMotor positionalMotor) {
        super(schedulerProvider.getScheduler(), key, new DistanceBasedInputsAutoLogged());

        this.positionalMotor = positionalMotor;
    }

    @Override
    public void updateInputs() {
        inputs.currentPosition = positionalMotor.getPosition();
        inputs.targetPosition = positionalMotor.getTargetPosition().orElse(null);
    }

    @Override
    public void setCurrentPosition(Measure<Distance> position) {
        positionalMotor.setCurrentPosition(position);
    }

    @Override
    public void setTargetPosition(Measure<Distance> position) {
        positionalMotor.setTargetPosition(position);
    }

    @Override
    public void setPercentageOutput(double power) {
        positionalMotor.setPercentageOutput(power);
    }
}
