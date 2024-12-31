package redbacks.robot.scorpion.subsystems.climber;

import com.ctre.phoenix6.signals.NeutralModeValue;

import arachne4.lib.scheduler.SchedulerProvider;
import redbacks.lib.hardware.motor.ctre.SmartTalonFxBuilder;
import redbacks.lib.subsystems.positionbased.DistanceBasedMotorHardware;

public class ScorpionClimberHardware extends DistanceBasedMotorHardware {
    public ScorpionClimberHardware(SchedulerProvider schedulerProvider) {
        super(schedulerProvider, ScorpionClimberConstants.INPUTS_LOGGING_KEY, SmartTalonFxBuilder
            .fromId(14, NeutralModeValue.Brake)
            .withConfig(ScorpionClimberConstants.MOTOR_CONFIG)
            .buildUsingIndividualControl()
            .createEmpiricallyDistancedMotor(ScorpionClimberConstants.REVOLUTIONS_PER_METRE, ScorpionClimberConstants.PID_CONFIG, false));
    }
}
