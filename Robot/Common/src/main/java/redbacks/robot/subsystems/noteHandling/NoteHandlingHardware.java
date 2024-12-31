package redbacks.robot.subsystems.noteHandling;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import arachne4.lib.Constants;
import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import redbacks.lib.hardware.motor.SmartWheeledMotor;
import redbacks.lib.hardware.motor.ctre.SmartTalonFx;
import redbacks.lib.hardware.motor.ctre.SmartTalonFxBuilder;
import redbacks.robot.subsystems.noteHandling.NoteHandlingConstants.ShooterConstants;

public class NoteHandlingHardware extends NoteHandlingIO {
    private static final ShooterConstants SHOOTER_CONSTANTS = Constants.get(ShooterConstants.class);

    public NoteHandlingHardware(SchedulerProvider schedulerProvider) {
        super(schedulerProvider.getScheduler());
    }

    private final SmartWheeledMotor
        leftShooterMotor = createShooterMotor(6, true),
        rightShooterMotor = createShooterMotor(5, false);

    private final SmartTalonFx intakeMotor = SmartTalonFxBuilder
        .fromId(4, NeutralModeValue.Brake)
        .disableFieldOrientedControl()
        .buildUsingIndividualControl();

    private final DigitalInput noteInIntake = new DigitalInput(1);

    private final DigitalInput _noteInStow = new DigitalInput(0);
    private final BooleanSupplier noteInStow = () -> !_noteInStow.get();

    private static SmartWheeledMotor createShooterMotor(int id, boolean isReversed) {
        return SmartTalonFxBuilder
            .fromId(id, NeutralModeValue.Coast)
            .withConfig(isReversed ? SHOOTER_CONSTANTS.invertedMotorConfig() : SHOOTER_CONSTANTS.motorConfig())
            .buildUsingIndividualControl()
            .createWheeledMotor(SHOOTER_CONSTANTS.wheelDiameter(), SHOOTER_CONSTANTS.gearRatio(), SHOOTER_CONSTANTS.pidConfig(), false);
    }

    @Override
    public void updateInputs() {
        inputs.leftShooterVelocity = leftShooterMotor.getVelocity();
        inputs.rightShooterVelocity = rightShooterMotor.getVelocity();

        inputs.isNoteInIntake = noteInIntake.get();
        inputs.isNoteInStow = noteInStow.getAsBoolean();
    }

    @Override
    public void setLeftShooterTargetVelocity(Measure<Velocity<Distance>> speed) {
        leftShooterMotor.setTargetVelocity(speed);
    }

    @Override
    public void setRightShooterTargetVelocity(Measure<Velocity<Distance>> speed) {
        rightShooterMotor.setTargetVelocity(speed);
    }

    @Override
    public void setIntakePercentageOutput(double power) {
        intakeMotor.setPercentageOutput(power);
    }
}
