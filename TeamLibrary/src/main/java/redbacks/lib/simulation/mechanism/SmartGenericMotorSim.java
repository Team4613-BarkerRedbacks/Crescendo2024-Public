package redbacks.lib.simulation.mechanism;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import redbacks.lib.math.characteristics.PIDFController;
import redbacks.lib.math.characteristics.PidConfig;
import redbacks.lib.math.characteristics.PidfConfig;
import redbacks.lib.math.characteristics.ProfiledPidConfig;
import redbacks.lib.simulation.MotorSimConfig;

public class SmartGenericMotorSim {
    public static record Characteristics(
        MotorSimConfig motorConfig,
        ProfiledPidConfig profiledPositionControllerFactory,
        PidConfig positionControllerFactory,
        PidfConfig velocityControllerFactory,
        DoubleSupplier usableVoltage) {}

    private static enum ControlType {
        OPEN_LOOP,
        PROFILED_POSITION_CLOSED_LOOP,
        POSITION_CLOSED_LOOP,
        VELOCITY_CLOSED_LOOP
    }

    private final DCMotorSim sim;

    private final ProfiledPIDController profiledPositionPidController;
    private final PIDController positionPidController;
    private final PIDFController velocityPidController;
    private ControlType controlType = ControlType.OPEN_LOOP;

    private final Measure<Time> loopPeriod;
    private final DoubleSupplier usableVoltage;

    public SmartGenericMotorSim(Measure<Time> loopPeriod, Characteristics characteristics) {
        this.sim = new DCMotorSim(
            characteristics.motorConfig.motor(),
            characteristics.motorConfig.gearRatio().getReduction(),
            characteristics.motorConfig.momentOfInertiaJoulesKgMetersSquared());

        this.profiledPositionPidController = characteristics.profiledPositionControllerFactory != null ? characteristics.profiledPositionControllerFactory.createController(loopPeriod) : null;
        this.positionPidController = characteristics.positionControllerFactory != null ? characteristics.positionControllerFactory.createController(loopPeriod) : null;
        this.velocityPidController = characteristics.velocityControllerFactory != null ? characteristics.velocityControllerFactory.createController(loopPeriod) : null;

        this.loopPeriod = loopPeriod;
        this.usableVoltage = characteristics.usableVoltage;
    }

    public void update() {
        switch (controlType) {
            case PROFILED_POSITION_CLOSED_LOOP -> sim.setInputVoltage(clampToUsableVoltage(profiledPositionPidController.calculate(sim.getAngularPositionRad())));
            case POSITION_CLOSED_LOOP -> sim.setInputVoltage(clampToUsableVoltage(positionPidController.calculate(sim.getAngularPositionRad())));
            case VELOCITY_CLOSED_LOOP -> sim.setInputVoltage(clampToUsableVoltage(velocityPidController.calculate(sim.getAngularVelocityRadPerSec())));
            case OPEN_LOOP -> {}
        }

        sim.update(loopPeriod.in(Units.Seconds));
    }

    public void setCurrentAngle(Rotation2d angle) {
        sim.setState(angle.getRadians(), sim.getAngularVelocityRadPerSec());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngularPositionRad());
    }

    public Measure<Velocity<Angle>> getCurrentVelocity() {
        return Units.RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    }

    public void setTargetAngle(Rotation2d target) {
        if (profiledPositionPidController != null) {
            if (controlType != ControlType.PROFILED_POSITION_CLOSED_LOOP) profiledPositionPidController.reset(sim.getAngularPositionRad(), sim.getAngularVelocityRadPerSec());

            controlType = ControlType.PROFILED_POSITION_CLOSED_LOOP;
            profiledPositionPidController.setGoal(target.getRadians());
        }
        else {
            if (controlType != ControlType.POSITION_CLOSED_LOOP) positionPidController.reset();

            controlType = ControlType.POSITION_CLOSED_LOOP;
            positionPidController.setSetpoint(target.getRadians());
        }
    }

    public Optional<Rotation2d> getTargetAngle() {
        return Optional.ofNullable(switch (controlType) {
            case PROFILED_POSITION_CLOSED_LOOP -> Rotation2d.fromRadians(profiledPositionPidController.getGoal().position);
            case POSITION_CLOSED_LOOP -> Rotation2d.fromRadians(positionPidController.getSetpoint());
            default -> null;
        });
    }

    public void setTargetVelocity(Measure<Velocity<Angle>> target) {
        if (controlType != ControlType.VELOCITY_CLOSED_LOOP) velocityPidController.reset();

        controlType = ControlType.VELOCITY_CLOSED_LOOP;
        velocityPidController.setSetpoint(target.in(Units.RadiansPerSecond));
    }

    public Optional<Measure<Velocity<Angle>>> getTargetVelocity() {
        return controlType == ControlType.VELOCITY_CLOSED_LOOP
            ? Optional.of(Units.RadiansPerSecond.of(velocityPidController.getSetpoint()))
            : Optional.empty();
    }

    public void setPercentageOutput(double power) {
        controlType = ControlType.OPEN_LOOP;
        sim.setInputVoltage(MathUtil.clamp(power, -1, 1) * usableVoltage.getAsDouble());
    }

    private double clampToUsableVoltage(double input) {
        double usableVoltage = this.usableVoltage.getAsDouble();
        return MathUtil.clamp(input, -usableVoltage, usableVoltage);
    }
}
