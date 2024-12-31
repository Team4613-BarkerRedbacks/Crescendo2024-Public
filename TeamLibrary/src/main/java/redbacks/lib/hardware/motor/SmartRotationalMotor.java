package redbacks.lib.hardware.motor;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SmartRotationalMotor extends SmartMotor {
    Rotation2d getAngle();
    Optional<Rotation2d> getTargetAngle();

    void setSensorAngle(Rotation2d angle);
    void setTargetAngle(Rotation2d angle);
    void setTargetAngle(Rotation2d angle, double arbitraryFeedForwardPercent);
}
