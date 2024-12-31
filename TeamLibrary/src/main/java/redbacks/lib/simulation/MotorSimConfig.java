package redbacks.lib.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import redbacks.lib.math.characteristics.GearRatio;

public record MotorSimConfig(DCMotor motor, GearRatio gearRatio, double momentOfInertiaJoulesKgMetersSquared) {}
