package redbacks.lib.hardware.motor;

import java.util.Optional;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity; 

public interface SmartWheeledMotor extends SmartMotor {
    Measure<Distance> getPosition();
    Measure<Velocity<Distance>> getVelocity();

    Optional<Measure<Distance>> getTargetPosition();
    Optional<Measure<Velocity<Distance>>> getTargetVelocity();

    void setCurrentPosition(Measure<Distance> position);

    void setTargetPosition(Measure<Distance> position);
    void setTargetPosition(Measure<Distance> position, double arbitraryFeedForwardPercent);
    void setTargetVelocity(Measure<Velocity<Distance>> velocity);
    double getRawVelocity();
}
