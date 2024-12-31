package redbacks.robot.subsystems.drivetrain.behaviours;

import redbacks.robot.CommonConstants;
import redbacks.robot.subsystems.drivetrain.CommonDrivetrain;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;
import arachne4.lib.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AimAtSpeakerDriveBehaviour extends DrivetrainBehaviour {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

    private final CommonDrivetrain drivetrain;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(
    10, 0, 0.3,
        new TrapezoidProfile.Constraints(CONSTANTS.getAutoMaxRotationalVelocity().times(2), CONSTANTS.getAutoMaxRotationalAcceleration().times(4)),
        CommonConstants.LOOP_PERIOD.in(Units.Seconds)
    );

    public AimAtSpeakerDriveBehaviour(CommonDrivetrain drivetrain, DrivetrainIO io) {
        super(io);

        this.drivetrain = drivetrain;

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationController.setTolerance(3);
    }

    @Override
    public void onEnterMode() {
        rotationController.reset(drivetrain.getPosition().getRotation().getRadians(), drivetrain.getFieldRelativeVelocity().omegaRadiansPerSecond);
    }

    @Override
    public void acceptDriverInputs(double forward, double left, double rotate) {
        Rotation2d currentAngle = drivetrain.getPosition().getRotation();
        Rotation2d targetAngle = drivetrain.getVelocityOffsetSpeakerPosition().getAngle();

        rotationController.setGoal(targetAngle.getRadians() - Math.toRadians(1));

        SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("aim power", Math.toDegrees(rotationController.calculate(currentAngle.getRadians())));

        io.driveWithVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                CONSTANTS.getMaxLinearVelocity().times(forward),
                CONSTANTS.getMaxLinearVelocity().times(left),
                Units.RadiansPerSecond.of(rotationController.calculate(currentAngle.getRadians())),
                currentAngle
            )
        );

        io.logTargetPosition(new Pose2d(io.inputs.position.getTranslation(), targetAngle));
    }

    @Override
    public void onLeaveMode() {
        io.logTargetPosition(null);
    }
}
