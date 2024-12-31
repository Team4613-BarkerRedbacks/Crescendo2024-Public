package redbacks.lib.hardware.mechanism.drivetrain.ctre;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.lib.hardware.mechanism.drivetrain.SmartSwerveDrivetrain;
import redbacks.lib.hardware.mechanism.drivetrain.ctre.copied.CopiedCtreSwerveDrivetrain;
import redbacks.lib.hardware.mechanism.drivetrain.ctre.copied.CopiedCtreSwerveRequest;

public class SmartCtreSwerveDrivetrain extends SchedulerProviderBase implements SmartSwerveDrivetrain {
    private final VisibleDrivetrain drivetrain;

    public SmartCtreSwerveDrivetrain(
            Scheduler scheduler,
            SwerveDrivetrainConstants drivetrainConstants,
            Pigeon2Configuration pigeon2Configuration,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants... modules) {
        super(scheduler);

        this.drivetrain = new VisibleDrivetrain(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        drivetrain.getPigeon2().getConfigurator().apply(pigeon2Configuration);
    }

    { registerHandler(Scheduler.EXECUTE, this::printPigeon);}
    private void printPigeon() {
        SmartDashboard.putNumber("Raw Pigeon", drivetrain.getPigeon2().getAngle());
    }

    // ----------------------------------------
    // Requests
    // ----------------------------------------

    private final CopiedCtreSwerveRequest.ApplyChassisSpeeds chassisSpeedsRequest = new CopiedCtreSwerveRequest.ApplyChassisSpeeds();
        // .withDriveRequestType(DriveRequestType.Velocity)
        // .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    @Override
    public void driveWithVelocity(ChassisSpeeds speeds) {
        drivetrain.setControl(chassisSpeedsRequest.withSpeeds(speeds));
    }

    // ----------------------------------------
    // Odometry
    // ----------------------------------------

    @Override
    public Pose2d getPosition() {
        return drivetrain.getState().Pose;
    }

    @Override
    public void setPosition(Pose2d position) {
        drivetrain.seedFieldRelative(position);
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return drivetrain.getKinematics();
    }

    @Override
    public void addVisionMeasurement(Pose2d visionCalculatedPositionMetres, double timestampSeconds) {
        drivetrain.addVisionMeasurement(visionCalculatedPositionMetres, timestampSeconds);
    }

    // ----------------------------------------
    // Velocity calculation and caching
    // ----------------------------------------

    private ChassisSpeeds fieldRelativeVelocities;

    { registerHandler(Scheduler.PRE_EXECUTE, this::updateFieldRelativeVelocities); }
    private void updateFieldRelativeVelocities() {
        fieldRelativeVelocities = ChassisSpeeds.fromFieldRelativeSpeeds(
            drivetrain.getState().speeds,
            drivetrain.getState().Pose.getRotation().unaryMinus());
    }

    @Override
    public ChassisSpeeds getVelocityMetresAndRadiansPerSec() {
        return fieldRelativeVelocities;
    }

    // ----------------------------------------
    // Telemetry
    // ----------------------------------------

    @Override
    public void telemetry() {
        SmartDashboard.putNumber("FL Module Angle", drivetrain.getModule(0).getPosition(false).angle.getDegrees());
        SmartDashboard.putNumber("BL Module Angle", drivetrain.getModule(1).getPosition(false).angle.getDegrees());
        SmartDashboard.putNumber("BR Module Angle", drivetrain.getModule(2).getPosition(false).angle.getDegrees());
        SmartDashboard.putNumber("FR Module Angle", drivetrain.getModule(3).getPosition(false).angle.getDegrees());
    }

    // ----------------------------------------
    // Private vendor drivetrain subclass to access kinematics
    // ----------------------------------------

    private static final class VisibleDrivetrain extends CopiedCtreSwerveDrivetrain {
        private VisibleDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants... modules) {
            super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        }

        private SwerveDriveKinematics getKinematics() {
            return m_kinematics;
        }
    }
}
