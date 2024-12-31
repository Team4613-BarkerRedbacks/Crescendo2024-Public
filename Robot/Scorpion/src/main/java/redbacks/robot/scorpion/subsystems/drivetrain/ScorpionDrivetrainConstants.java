package redbacks.robot.scorpion.subsystems.drivetrain;

import java.util.Map;

import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import arachne4.lib.Constants.RobotConstants;
import arachne4.lib.math.Interpolators;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.hardware.mechanism.drivetrain.SmartSwerveDrivetrain.SwerveDrivePoseEstimatorConfig;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants;

@RobotConstants(Scorpion.class)
public class ScorpionDrivetrainConstants implements DrivetrainConstants {
    // Controls
	private final double
        JOYSTICK_DEADZONE_RADIUS = 0.1,
        JOYSTICK_EXPONENT_FOR_LINEAR = 2.5,
        JOYSTICK_EXPONENT_FOR_ROTATION = 2;

    // Modules
    private final Measure<Distance>
        DISTANCE_BETWEEN_MODULE_X = Units.Meters.of(0.47665),
        DISTANCE_BETWEEN_MODULE_Y = Units.Meters.of(0.47665),
        MODULE_DISTANCE_FROM_CENTRE_X = DISTANCE_BETWEEN_MODULE_X.divide(2),
        MODULE_DISTANCE_FROM_CENTRE_Y = DISTANCE_BETWEEN_MODULE_Y.divide(2);

    private final Measure<Distance> WHEEL_DIAMETER = Units.Millimeters.of(96);

    private final GearRatio
        DRIVE_GEAR_RATIO = new GearRatio(5.357143, 1),
        STEER_GEAR_RATIO = new GearRatio(150, 7);

    private final Rotation2d
        FRONT_LEFT_OFFSET = Rotation2d.fromDegrees(-43.9 + 180 - 1.6),
        BACK_LEFT_OFFSET = Rotation2d.fromDegrees(-9.3 + 180),
        BACK_RIGHT_OFFSET = Rotation2d.fromDegrees(77.2 - 180),
        FRONT_RIGHT_OFFSET = Rotation2d.fromDegrees(-237 + 180 + 1.9);

    // Drivetrain
    private final Measure<Velocity<Distance>> MAX_LINEAR_VELOCITY = Units.MetersPerSecond.of(6.1);
    private final Measure<Velocity<Distance>> AUTO_SWEEP_VELOCITY = Units.MetersPerSecond.of(3);
    private final Measure<Velocity<Angle>> MAX_ROTATIONAL_VELOCITY = Units.RotationsPerSecond.of(1.42);
    private final Measure<Velocity<Angle>> MAX_ROTATIONAL_VELOCITY_WHILE_MOVING = Units.RotationsPerSecond.of(2);
    private final InterpolatingTreeMap<Measure<Velocity<Distance>>, Measure<Velocity<Angle>>> LINEAR_SPEED_TO_ROTATIONAL_MAX_SPEED = Interpolators.interpolateOver(
        Interpolators.forMeasure(), Interpolators.forMeasure(), Map.of(
            Units.MetersPerSecond.zero(), MAX_ROTATIONAL_VELOCITY,
            MAX_LINEAR_VELOCITY.divide(0.5), MAX_ROTATIONAL_VELOCITY_WHILE_MOVING));

    // Dimensions
    private static final Measure<Distance> ROBOT_WIDTH = Units.Meters.of(0.771); // Including 17cm bumpers
    private static final Measure<Distance> ROBOT_LENGTH = Units.Meters.of(0.771); // Including 17cm bumpers

    public static final Measure<Distance>
        ROBOT_OFFSET_X_FROM_EDGE = ROBOT_LENGTH.divide(2),
        ROBOT_OFFSET_Y_FROM_EDGE = ROBOT_WIDTH.divide(2);

    private final SwerveDrivePoseEstimatorConfig POSE_ESTIMATOR_CONFIG = new SwerveDrivePoseEstimatorConfig(
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(5, 5, Double.POSITIVE_INFINITY)
    );

    private final double
        LINEAR_KP = 2.2,
        LINEAR_KI = 0.2,
        LINEAR_KD = 0.4;

    private final double ROTATIONAL_KP = 10;

    private final double ROTATIONAL_KI = 0;

    private final double ROTATIONAL_KD = 0;

    private final Measure<Velocity<Angle>> AUTO_MAX_ROTATIONAL_VELOCITY = Units.RotationsPerSecond.of(5);
    private final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION = MAX_LINEAR_VELOCITY.per(Units.Second);
    private final Measure<Velocity<Velocity<Angle>>> AUTO_MAX_ROTATIONAL_ACCELERATION = AUTO_MAX_ROTATIONAL_VELOCITY.per(Units.Second);

    private final double
        POSITION_TOLERANCE_METRES = 0.2,
        ROTATION_TOLERANCE_RADIANS = Math.toRadians(1.5);
        
    private final TrapezoidProfile.Constraints
        AUTO_ROTATION_MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(AUTO_MAX_ROTATIONAL_VELOCITY, AUTO_MAX_ROTATIONAL_ACCELERATION);

    private final double ROTATION_LERP_LEAD_PERCENTAGE = 0.2;

    private final Slot0Configs driveGains = new Slot0Configs()
        .withKP(2.4e-5 * 0.2)
        .withKD(3.5e-4 * 0.0002)
        .withKV(0.0467 * 0.2);

    private final Slot0Configs steerGains = new Slot0Configs()
        .withKP(0.25 * STEER_GEAR_RATIO.getReduction());

    private final Pigeon2Configuration pigeonConfig = new Pigeon2Configuration()
        .withPigeon2Features(new Pigeon2FeaturesConfigs()
            .withEnableCompass(false))
        .withMountPose(new MountPoseConfigs()
            .withMountPosePitch(-0.80)
            .withMountPoseRoll(-0.05))
        .withGyroTrim(new GyroTrimConfigs()
            .withGyroScalarZ(0.81));

    @Override
    public double getJoystickDeadzoneRadius() {
        return JOYSTICK_DEADZONE_RADIUS;
    }

    @Override
    public double getJoystickExponentForLinear() {
        return JOYSTICK_EXPONENT_FOR_LINEAR;
    }

    @Override
    public double getJoystickExponentForRotation() {
        return JOYSTICK_EXPONENT_FOR_ROTATION;
    }

    // Module Constants
    @Override
    public Measure<Distance> getDistanceBetweenModuleX() {
        return DISTANCE_BETWEEN_MODULE_X;
    }

    @Override
    public Measure<Distance> getDistanceBetweenModuleY() {
        return DISTANCE_BETWEEN_MODULE_Y;
    }

    @Override
    public Measure<Distance> getModuleDistanceFromCentreX() {
        return MODULE_DISTANCE_FROM_CENTRE_X;
    }

    @Override
    public Measure<Distance> getModuleDistanceFromCentreY() {
        return MODULE_DISTANCE_FROM_CENTRE_Y;
    }

    @Override
    public Measure<Distance> getWheelDiameter() {
        return WHEEL_DIAMETER;
    }

    // Gear Ratio Constants
    @Override
    public GearRatio getDriveGearRatio() {
        return DRIVE_GEAR_RATIO;
    }

    @Override
    public GearRatio getSteerGearRatio() {
        return STEER_GEAR_RATIO;
    }

    // Rotation2d Constants
    @Override
    public Rotation2d getFrontLeftOffset() {
        return FRONT_LEFT_OFFSET;
    }

    @Override
    public Rotation2d getBackLeftOffset() {
        return BACK_LEFT_OFFSET;
    }

    @Override
    public Rotation2d getBackRightOffset() {
        return BACK_RIGHT_OFFSET;
    }

    @Override
    public Rotation2d getFrontRightOffset() {
        return FRONT_RIGHT_OFFSET;
    }

    // Drivetrain Constants
    @Override
    public Measure<Velocity<Distance>> getMaxLinearVelocity() {
        return MAX_LINEAR_VELOCITY;
    }

    @Override
    public Measure<Velocity<Distance>> getAutoSweepVelocity() {
        return AUTO_SWEEP_VELOCITY;
    }

    @Override
    public Measure<Velocity<Angle>> getMaxRotationalVelocity() {
        return MAX_ROTATIONAL_VELOCITY;
    }

    @Override
    public InterpolatingTreeMap<Measure<Velocity<Distance>>, Measure<Velocity<Angle>>> getLinearSpeedToRotationalMaxSpeed() {
        return LINEAR_SPEED_TO_ROTATIONAL_MAX_SPEED;
    }

    // Dimension Constants
    @Override
    public Measure<Distance> getRobotWidth() {
        return ROBOT_WIDTH;
    }

    @Override
    public Measure<Distance> getRobotLength() {
        return ROBOT_LENGTH;
    }

    @Override
    public Measure<Distance> getRobotOffsetXFromEdge() {
        return ROBOT_OFFSET_X_FROM_EDGE;
    }

    @Override
    public Measure<Distance> getRobotOffsetYFromEdge() {
        return ROBOT_OFFSET_Y_FROM_EDGE;
    }

    // Swerve Drive Pose Estimator Config Constants
    @Override
    public SwerveDrivePoseEstimatorConfig getPoseEstimatorConfig() {
        return POSE_ESTIMATOR_CONFIG;
    }

    // Constants
    @Override
    public double getLinearKp() {
        return LINEAR_KP;
    }

    @Override
    public double getLinearKi() {
        return LINEAR_KI;
    }

    @Override
    public double getLinearKd() {
        return LINEAR_KD;
    }

    @Override
    public double getRotationalKp() {
        return ROTATIONAL_KP;
    }

    @Override
    public double getRotationalKi() {
        return ROTATIONAL_KI;
    }

    @Override
    public double getRotationalKd() {
        return ROTATIONAL_KD;
    }

    @Override
    public Measure<Velocity<Angle>> getAutoMaxRotationalVelocity() {
        return AUTO_MAX_ROTATIONAL_VELOCITY;
    }

    @Override
    public Measure<Velocity<Velocity<Distance>>> getMaxAcceleration() {
        return MAX_ACCELERATION;
    }

    @Override
    public Measure<Velocity<Velocity<Angle>>> getAutoMaxRotationalAcceleration() {
        return AUTO_MAX_ROTATIONAL_ACCELERATION;
    }

    @Override
    public double getPositionToleranceMetres() {
        return POSITION_TOLERANCE_METRES;
    }

    @Override
    public double getRotationToleranceRadians() {
        return ROTATION_TOLERANCE_RADIANS;
    }

    // Trapezoid Profile Constraints Constants
    @Override
    public TrapezoidProfile.Constraints getAutoRotationMotionConstraints() {
        return AUTO_ROTATION_MOTION_CONSTRAINTS;
    }

    @Override
    public double getRotationLerpLeadPercentage() {
        return ROTATION_LERP_LEAD_PERCENTAGE;
    }

    // TalonFX Config Constants
    @Override
    public Slot0Configs getDriveGains() {
        return driveGains;
    }

    @Override
    public Slot0Configs getSteerGains() {
        return steerGains;
    }

    // Pigeon Config
    @Override
    public Pigeon2Configuration getPigeonConfig() {
        return pigeonConfig;
    }
}