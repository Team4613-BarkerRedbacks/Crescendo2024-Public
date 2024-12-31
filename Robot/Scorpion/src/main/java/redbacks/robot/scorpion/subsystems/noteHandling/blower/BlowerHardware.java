package redbacks.robot.scorpion.subsystems.noteHandling.blower;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import arachne4.lib.scheduler.SchedulerProvider;
import redbacks.lib.hardware.motor.ctre.SmartTalonFxBuilder;
import redbacks.lib.subsystems.positionbased.AngleBasedMotorHardware;

public class BlowerHardware extends AngleBasedMotorHardware implements BlowerIO {
    public BlowerHardware(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, KEY, SmartTalonFxBuilder
            .fromId(9, NeutralModeValue.Coast)
            .withConfig(BlowerConstants.ARM_MOTOR_CONFIG)
            .disableFieldOrientedControl()
            .buildUsingIndividualControl()
            .createRotationalMotor(BlowerConstants.ARM_GEAR_RATIO, BlowerConstants.ARM_PID_CONFIG, true));
    }

    private final WPI_VictorSPX blowMotor = new WPI_VictorSPX(7);

    @Override
    public void blowWithPercentageOutput(double power) {
        blowMotor.set(power);
    }
}