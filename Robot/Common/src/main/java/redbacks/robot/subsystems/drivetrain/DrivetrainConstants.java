package redbacks.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import redbacks.lib.hardware.mechanism.drivetrain.SmartSwerveDrivetrain.SwerveDrivePoseEstimatorConfig;
import redbacks.lib.math.characteristics.GearRatio;

public interface DrivetrainConstants {
    String INPUTS_LOGGING_KEY = "Drivetrain";

    double getJoystickDeadzoneRadius();

    double getJoystickExponentForLinear();

    double getJoystickExponentForRotation();

    // Module Constants
    Measure<Distance> getDistanceBetweenModuleX();

    Measure<Distance> getDistanceBetweenModuleY();

    Measure<Distance> getModuleDistanceFromCentreX();

    Measure<Distance> getModuleDistanceFromCentreY();

    Measure<Distance> getWheelDiameter();

    // Gear Ratio Constants
    GearRatio getDriveGearRatio();

    GearRatio getSteerGearRatio();

    // Rotation2d Constants
    Rotation2d getFrontLeftOffset();

    Rotation2d getBackLeftOffset();

    Rotation2d getBackRightOffset();

    Rotation2d getFrontRightOffset();

    // Drivetrain Constants
    Measure<Velocity<Distance>> getMaxLinearVelocity();

    Measure<Velocity<Distance>> getAutoSweepVelocity();

    Measure<Velocity<Angle>> getMaxRotationalVelocity();

    InterpolatingTreeMap<Measure<Velocity<Distance>>, Measure<Velocity<Angle>>> getLinearSpeedToRotationalMaxSpeed();

    // Dimension Constants
    Measure<Distance> getRobotWidth();

    Measure<Distance> getRobotLength();

    Measure<Distance> getRobotOffsetXFromEdge();

    Measure<Distance> getRobotOffsetYFromEdge();

    // Swerve Drive Pose Estimator Config Constants
    SwerveDrivePoseEstimatorConfig getPoseEstimatorConfig();

    // Pigeon Config
    Pigeon2Configuration getPigeonConfig();

    // Constants
    double getLinearKp();

    double getLinearKi();

    double getLinearKd();

    double getRotationalKp();

    double getRotationalKi();

    double getRotationalKd();

    Measure<Velocity<Angle>> getAutoMaxRotationalVelocity();

    Measure<Velocity<Velocity<Distance>>> getMaxAcceleration();

    Measure<Velocity<Velocity<Angle>>> getAutoMaxRotationalAcceleration();

    double getPositionToleranceMetres();

    double getRotationToleranceRadians();

    // Trapezoid Profile Constraints Constants
    TrapezoidProfile.Constraints getAutoRotationMotionConstraints();

    double getRotationLerpLeadPercentage();

    // TalonFX Config Constants
    Slot0Configs getDriveGains();
    Slot0Configs getSteerGains();
}
