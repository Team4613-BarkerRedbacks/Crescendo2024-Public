package redbacks.robot.scorpion.subsystems.drivetrain.behaviours;

import arachne4.lib.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.field.FieldLocations;
import redbacks.robot.CommonConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;
import redbacks.robot.subsystems.drivetrain.behaviours.DrivetrainBehaviour;

public class SnapToTrapBehaviour extends DrivetrainBehaviour {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

    final ProfiledPIDController yController = new ProfiledPIDController(
        10, 0, 0.3, 
        new TrapezoidProfile.Constraints(CONSTANTS.getMaxLinearVelocity(), CONSTANTS.getMaxAcceleration()),
        CommonConstants.LOOP_PERIOD.in(Units.Seconds));

    final ProfiledPIDController rotationController = new ProfiledPIDController(
        15, 0, 0.3,
        new TrapezoidProfile.Constraints(CONSTANTS.getAutoMaxRotationalVelocity().times(2), CONSTANTS.getAutoMaxRotationalAcceleration().times(4)),
        CommonConstants.LOOP_PERIOD.in(Units.Seconds));

    public SnapToTrapBehaviour(DrivetrainIO io) {
        super(io);

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void onEnterMode() {
        Rotation2d targetAngle = getTrapTag().getRotation().toRotation2d();
        rotationController.reset(io.inputs.position.getRotation().getRadians(), io.inputs.velocityMetresAndRadiansPerSec.omegaRadiansPerSecond);
        rotationController.setGoal(targetAngle.getRadians());
        yController.reset(getRobotOffsetFromTrap().getY());
        yController.setGoal(0);

    }

    @Override
    public void acceptDriverInputs(double forward, double left, double rotate) {
        double yErrorMeters = getRobotOffsetFromTrap().getY();

        io.driveWithVelocity(
            new ChassisSpeeds(
                CONSTANTS.getMaxLinearVelocity().in(Units.MetersPerSecond) * forward,
                yController.calculate(yErrorMeters),
                rotationController.calculate(io.inputs.position.getRotation().getRadians())
            )
        );

        io.logTargetPosition(new Pose2d(
            io.inputs.position.getTranslation().minus(new Translation2d(0, yErrorMeters).rotateBy(io.inputs.position.getRotation())),
            Rotation2d.fromRadians(rotationController.getGoal().position)
        ));
    }

    @Override
    public void onLeaveMode() {
        io.logTargetPosition(null);
    }

    private Pose3d getTrapTag() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        return alliance == Alliance.Red
            ? FieldLocations.APRIL_TAGS.get(11).in(alliance)
            : FieldLocations.APRIL_TAGS.get(16).in(alliance);
    }

    private Translation2d getRobotOffsetFromTrap() {
        Pose3d trapTag = getTrapTag();

        return io.inputs.position.getTranslation()
            .minus(trapTag.getTranslation().toTranslation2d())
            .rotateBy(trapTag.getRotation().toRotation2d().unaryMinus());
    }
}
