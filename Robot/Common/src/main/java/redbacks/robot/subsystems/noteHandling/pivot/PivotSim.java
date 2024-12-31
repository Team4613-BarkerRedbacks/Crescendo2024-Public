package redbacks.robot.subsystems.noteHandling.pivot;

import arachne4.lib.Constants;
import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.io.InputsProvider;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.ProfiledPidConfig;
import redbacks.lib.simulation.MotorSimConfig;
import redbacks.lib.simulation.mechanism.SmartSingleJointedArmSim;
import redbacks.lib.simulation.mechanism.SmartSingleJointedArmSim.ArmDimensions;
import redbacks.lib.subsystems.positionbased.AngleBasedIO;
import redbacks.lib.subsystems.positionbased.AngleBasedInputsAutoLogged;
import redbacks.robot.CommonConstants;

public class PivotSim extends InputsProvider<AngleBasedInputsAutoLogged> implements AngleBasedIO {
    private static final PivotConstants CONSTANTS = Constants.get(PivotConstants.class);
    private static final SimConstants SIM_CONSTANTS = CONSTANTS.simConstants();

    public static record SimConstants(
        GearRatio gearRatio,
        Measure<Velocity<Angle>> cruiseVelocity,
        Measure<Velocity<Velocity<Angle>>> maxAcceleration,
        Rotation2d hardStopAngle,
        Rotation2d maximumAngle) {}

    private final SmartSingleJointedArmSim sim = new SmartSingleJointedArmSim(
        CommonConstants.LOOP_PERIOD,
        new SmartSingleJointedArmSim.Characteristics(
            new MotorSimConfig(DCMotor.getFalcon500(1), SIM_CONSTANTS.gearRatio(), 0.3 /* Guessed */),
            new ProfiledPidConfig(100, 0, 0, new Constraints(SIM_CONSTANTS.cruiseVelocity(), SIM_CONSTANTS.maxAcceleration()), null),
            new ArmDimensions(Units.Meters.of(0.3) /* Guessed */, SIM_CONSTANTS.hardStopAngle(), SIM_CONSTANTS.maximumAngle()),
            () -> 12,
            true),
        SIM_CONSTANTS.hardStopAngle());

    public PivotSim(Scheduler scheduler) {
        super(scheduler, PivotConstants.INPUTS_LOGGING_KEY, new AngleBasedInputsAutoLogged());
    }

    @Override
    public void updateInputs() {
        sim.update();

        inputs.currentPosition = sim.getCurrentAngle();
        inputs.targetPosition = sim.getTargetAngle().orElse(null);
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
    public void setPercentageOutput(double power) {
        sim.setPercentageOutput(power);
    }

    public Rotation2d getAngle() {
        return sim.getCurrentAngle();
    }
}
