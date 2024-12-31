package redbacks.robot.scorpion.subsystems.noteHandling.blower;

import static redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerConstants.*;

import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import redbacks.lib.io.InputsProvider;
import redbacks.lib.math.characteristics.ProfiledPidConfig;
import redbacks.lib.simulation.MotorSimConfig;
import redbacks.lib.simulation.mechanism.SmartSingleJointedArmSim;
import redbacks.lib.simulation.mechanism.SmartSingleJointedArmSim.ArmDimensions;
import redbacks.lib.subsystems.positionbased.AngleBasedInputsAutoLogged;
import redbacks.robot.CommonConstants;

public class BlowerSim extends InputsProvider<AngleBasedInputsAutoLogged> implements BlowerIO {
    private final SmartSingleJointedArmSim sim = new SmartSingleJointedArmSim(
        CommonConstants.LOOP_PERIOD,
        new SmartSingleJointedArmSim.Characteristics(
            new MotorSimConfig(DCMotor.getFalcon500(1), ARM_GEAR_RATIO, 0.1 /* Guessed */),
            new ProfiledPidConfig(100, 0, 0, new Constraints(ARM_CRUISE_VELOCITY, ARM_MAX_ACCELERATION), null),
            new ArmDimensions(Units.Meters.of(0.6) /* Guessed */, Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(135)),
            () -> 12,
            true),
        Rotation2d.fromDegrees(0));

    public BlowerSim(Scheduler scheduler) {
        super(scheduler, KEY, new AngleBasedInputsAutoLogged());
    }

    @Override
    public void updateInputs() {
        sim.update();

        inputs.currentPosition = sim.getCurrentAngle();
        inputs.targetPosition = sim.getTargetAngle().orElse(null);
    }

    @Override
    public void setPercentageOutput(double percent) {
        sim.setPercentageOutput(percent);
    }

    @Override
    public void setCurrentPosition(Rotation2d angle) {
        sim.setCurrentAngle(angle);
    }

    @Override
    public void setTargetPosition(Rotation2d angle) {
        sim.setTargetAngle(angle);
    }

    @Override
    public void blowWithPercentageOutput(double power) { /* Do nothing */ }
}
