package redbacks.lib.math.characteristics;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;

public record PidConfig(double kP, double kI, double kD, ContinuousInputBounds bounds) implements ControllerFactory<PIDController> {
    public static record ContinuousInputBounds(double min, double max) {}

    public PIDController createController(Measure<Time> loopPeriod) {
        var controller = new PIDController(kP, kI, kD, loopPeriod.in(Units.Seconds));
        if (bounds != null) controller.enableContinuousInput(bounds.min, bounds.max);
        return controller;
    }
}

