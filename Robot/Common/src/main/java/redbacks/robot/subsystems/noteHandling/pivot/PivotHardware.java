package redbacks.robot.subsystems.noteHandling.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import arachne4.lib.Constants;
import arachne4.lib.scheduler.SchedulerProvider;
import redbacks.lib.hardware.motor.ctre.SmartTalonFxBuilder;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;
import redbacks.lib.subsystems.positionbased.AngleBasedMotorHardware;

public class PivotHardware extends AngleBasedMotorHardware {
    private static final PivotConstants CONSTANTS = Constants.get(PivotConstants.class);

    public static record HardwareConstants(
        int talonId,
        TalonFXConfiguration motorConfig,
        GearRatio gearRatio,
        PidfConfig pidConfig) {}

    public PivotHardware(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, PivotConstants.INPUTS_LOGGING_KEY, SmartTalonFxBuilder
            .fromId(CONSTANTS.hardwareConstants().talonId, NeutralModeValue.Coast)
            .withConfig(CONSTANTS.hardwareConstants().motorConfig)
            .disableFieldOrientedControl()
            .buildUsingIndividualControl()
            .createRotationalMotor(CONSTANTS.hardwareConstants().gearRatio, CONSTANTS.hardwareConstants().pidConfig, true));
    }
}