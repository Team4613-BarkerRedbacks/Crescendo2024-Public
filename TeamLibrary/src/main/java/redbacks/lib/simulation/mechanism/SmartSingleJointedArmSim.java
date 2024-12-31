package redbacks.lib.simulation.mechanism;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import redbacks.lib.math.characteristics.ProfiledPidConfig;
import redbacks.lib.simulation.MotorSimConfig;

public class SmartSingleJointedArmSim {
    public static record Characteristics(
        MotorSimConfig motorConfig,
        ProfiledPidConfig pidConfig,
        ArmDimensions dimensions,
        DoubleSupplier usableVoltage,
        boolean simulateGravity) {}

    public static record ArmDimensions(Measure<Distance> armLength, Rotation2d minimumAngle, Rotation2d maximumAngle) {}

    private final SingleJointedArmSim sim;

    private final ProfiledPIDController pidController;
    private boolean pidControllerEnabled = false;

    private final Measure<Time> loopPeriod;
    private final DoubleSupplier usableVoltage;

    public SmartSingleJointedArmSim(Measure<Time> loopPeriod, Characteristics characteristics, Rotation2d initialAngle) {
        this.sim = new SingleJointedArmSim(
            characteristics.motorConfig.motor(),
            characteristics.motorConfig.gearRatio().getReduction(),
            characteristics.motorConfig.momentOfInertiaJoulesKgMetersSquared(),
            characteristics.dimensions.armLength.in(Units.Meters),
            characteristics.dimensions.minimumAngle.getRadians(),
            characteristics.dimensions.maximumAngle.getRadians(),
            characteristics.simulateGravity,
            initialAngle.getRadians());

        this.pidController = characteristics.pidConfig.createController(loopPeriod);
        this.loopPeriod = loopPeriod;
        this.usableVoltage = characteristics.usableVoltage;
    }

    public void update() {
        if (pidControllerEnabled) sim.setInputVoltage(clampToUsableVoltage(pidController.calculate(sim.getAngleRads())));

        sim.update(loopPeriod.in(Units.Seconds));
    }

    public void setCurrentAngle(Rotation2d angle) {
        sim.setState(angle.getRadians(), sim.getVelocityRadPerSec());
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads());
    }

    public void setTargetAngle(Rotation2d target) {
        pidController.setGoal(target.getRadians());
        pidControllerEnabled = true;
    }

    public Optional<Rotation2d> getTargetAngle() {
        return pidControllerEnabled ? Optional.of(Rotation2d.fromRadians(pidController.getGoal().position)) : Optional.empty();
    }

    public void setPercentageOutput(double power) {
        pidControllerEnabled = false;
        sim.setInputVoltage(MathUtil.clamp(power, -1, 1) * usableVoltage.getAsDouble());
    }

    private double clampToUsableVoltage(double input) {
        double usableVoltage = this.usableVoltage.getAsDouble();
        return MathUtil.clamp(input, -usableVoltage, usableVoltage);
    }
}
