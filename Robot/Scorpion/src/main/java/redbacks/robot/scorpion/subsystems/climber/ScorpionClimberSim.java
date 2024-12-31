package redbacks.robot.scorpion.subsystems.climber;

import static redbacks.robot.scorpion.subsystems.climber.ScorpionClimberConstants.*;

import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import redbacks.lib.io.InputsProvider;
import redbacks.lib.subsystems.positionbased.DistanceBasedIO;
import redbacks.lib.subsystems.positionbased.DistanceBasedInputsAutoLogged;
import redbacks.robot.CommonConstants;

public class ScorpionClimberSim extends InputsProvider<DistanceBasedInputsAutoLogged> implements DistanceBasedIO {
    private final DCMotorSim sim = new DCMotorSim(
        DCMotor.getKrakenX60Foc(1),
        GEAR_RATIO.getReduction(),
        0.001);

    private final PIDController pidController = new PIDController(100, 0, 0);
    private boolean pidControllerEnabled = false;

    public ScorpionClimberSim(Scheduler scheduler) {
        super(scheduler, ScorpionClimberConstants.INPUTS_LOGGING_KEY, new DistanceBasedInputsAutoLogged());
    }

    @Override
    public void updateInputs() {
        if (pidControllerEnabled) {
            sim.setInputVoltage(MathUtil.clamp(pidController.calculate(sim.getAngularPositionRad()), -12, 12));
        }

        sim.update(CommonConstants.LOOP_PERIOD.in(Units.Seconds));

        inputs.currentPosition = Units.Meters.of(sim.getAngularPositionRotations() * GEAR_RATIO.getReduction() / REVOLUTIONS_PER_METRE);
        inputs.targetPosition = pidControllerEnabled ? Units.Meters.of(pidController.getSetpoint() * 2 * Math.PI * GEAR_RATIO.getReduction() / REVOLUTIONS_PER_METRE) : null;
    }

    @Override
    public void setPercentageOutput(double percent) {
        sim.setInputVoltage(MathUtil.clamp(percent, -1, 1) * 12);
        pidControllerEnabled = false;
    }

    @Override
    public void setCurrentPosition(Measure<Distance> height) {
        sim.setState(height.in(Units.Meters) * REVOLUTIONS_PER_METRE * 2 * Math.PI / GEAR_RATIO.getReduction(), sim.getAngularVelocityRadPerSec());
    }

    @Override
    public void setTargetPosition(Measure<Distance> target) {
        pidController.setSetpoint(target.in(Units.Meters) * REVOLUTIONS_PER_METRE * 2 * Math.PI / GEAR_RATIO.getReduction());
        pidControllerEnabled = true;
    }
}
