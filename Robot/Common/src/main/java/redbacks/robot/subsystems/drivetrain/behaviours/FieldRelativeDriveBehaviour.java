package redbacks.robot.subsystems.drivetrain.behaviours;

import arachne4.lib.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;

public class FieldRelativeDriveBehaviour extends DrivetrainBehaviour {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

    public FieldRelativeDriveBehaviour(DrivetrainIO io) {
        super(io);
    }

    @Override
    public void acceptDriverInputs(double forward, double left, double rotate) {
        var linearVelocityX = CONSTANTS.getMaxLinearVelocity().times(forward);
        var linearVelocityY = CONSTANTS.getMaxLinearVelocity().times(left);

        var linearVelocity = Units.MetersPerSecond.of(Math.hypot(
            linearVelocityX.in(Units.MetersPerSecond),
            linearVelocityY.in(Units.MetersPerSecond)));

        io.driveWithVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                CONSTANTS.getMaxLinearVelocity().times(forward),
                CONSTANTS.getMaxLinearVelocity().times(left),
                CONSTANTS.getLinearSpeedToRotationalMaxSpeed().get(linearVelocity).times(rotate),
                io.inputs.position.getRotation()
            )
        );
    }
}
