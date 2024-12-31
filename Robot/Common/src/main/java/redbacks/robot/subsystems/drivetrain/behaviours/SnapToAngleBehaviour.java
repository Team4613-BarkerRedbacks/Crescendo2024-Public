package redbacks.robot.subsystems.drivetrain.behaviours;

import java.util.function.Supplier;

import arachne4.lib.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import redbacks.robot.CommonConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;

public class SnapToAngleBehaviour extends DrivetrainBehaviour {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

    final ProfiledPIDController rotationController = new ProfiledPIDController(
        15, 0, 0.3,
        new TrapezoidProfile.Constraints(CONSTANTS.getAutoMaxRotationalVelocity().times(2), CONSTANTS.getAutoMaxRotationalAcceleration().times(4)),
        CommonConstants.LOOP_PERIOD.in(Units.Seconds)
        );

    final Supplier<Rotation2d> targetAngle;

    public SnapToAngleBehaviour(DrivetrainIO io, Rotation2d targetAngle) {
        super(io);

        this.targetAngle = () -> targetAngle;
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SnapToAngleBehaviour(DrivetrainIO io, Supplier<Rotation2d> targetAngle) {
        super(io);

        this.targetAngle = targetAngle;
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void onEnterMode() {
        rotationController.reset(io.inputs.position.getRotation().getRadians(), io.inputs.velocityMetresAndRadiansPerSec.omegaRadiansPerSecond);
        rotationController.setGoal(targetAngle.get().getRadians());
    }

    @Override
    public void acceptDriverInputs(double forward, double left, double rotate) {
        Rotation2d currentTarget = targetAngle.get();

        rotationController.setGoal(currentTarget.getRadians());
        io.driveWithVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                CONSTANTS.getMaxLinearVelocity().in(Units.MetersPerSecond) * forward,
                CONSTANTS.getMaxLinearVelocity().in(Units.MetersPerSecond) * left,
                rotationController.calculate(io.inputs.position.getRotation().getRadians()),
                io.inputs.position.getRotation()
            )
        );

        io.logTargetPosition(new Pose2d(io.inputs.position.getTranslation(), currentTarget));
    }

    @Override
    public void onLeaveMode() {
        io.logTargetPosition(null);
    }
}
