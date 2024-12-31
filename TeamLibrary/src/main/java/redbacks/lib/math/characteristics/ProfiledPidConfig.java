package redbacks.lib.math.characteristics;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import redbacks.lib.math.characteristics.PidConfig.ContinuousInputBounds;

public record ProfiledPidConfig(double kP, double kI, double kD, TrapezoidProfile.Constraints constraints, ContinuousInputBounds bounds) implements ControllerFactory<ProfiledPIDController> {
    public ProfiledPIDController createController(Measure<Time> loopPeriod) {
        var controller = new ProfiledPIDController(kP, kI, kD, constraints, loopPeriod.in(Units.Seconds));
        if (bounds != null) controller.enableContinuousInput(bounds.min(), bounds.max());
        return controller;
    }
}
