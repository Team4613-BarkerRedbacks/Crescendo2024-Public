package redbacks.lib.subsystems.positionbased;

import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.math.geometry.Rotation2d;
import redbacks.lib.hardware.motor.SmartRotationalMotor;
import redbacks.lib.io.InputsProvider;

public class AngleBasedMotorHardware extends InputsProvider<AngleBasedInputsAutoLogged> implements AngleBasedIO {
    protected final SmartRotationalMotor positionalMotor;

    public AngleBasedMotorHardware(SchedulerProvider schedulerProvider, String key, SmartRotationalMotor positionalMotor) {
        super(schedulerProvider.getScheduler(), key, new AngleBasedInputsAutoLogged());

        this.positionalMotor = positionalMotor;
    }

    @Override
    public AngleBasedInputsAutoLogged inputs() {
        return inputs;
    }

    @Override
    public void updateInputs() {
        inputs.currentPosition = positionalMotor.getAngle();
        inputs.targetPosition = positionalMotor.getTargetAngle().orElse(null);
    }

    @Override
    public void setCurrentPosition(Rotation2d angle) {
        positionalMotor.setSensorAngle(angle);
    }

    @Override
    public void setTargetPosition(Rotation2d angle) {
        positionalMotor.setTargetAngle(angle);
    }

    @Override
    public void setPercentageOutput(double power) {
        positionalMotor.setPercentageOutput(power);
    }
}
