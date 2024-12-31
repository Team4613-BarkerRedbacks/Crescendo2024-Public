package redbacks.robot.subsystems.drivetrain;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Units;
import arachne4.lib.Constants;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import redbacks.lib.hardware.mechanism.drivetrain.SmartSwerveDrivetrain;
import redbacks.lib.hardware.mechanism.drivetrain.ctre.SmartCtreSwerveDrivetrain;
import redbacks.robot.CommonConstants;

public class DrivetrainHardware extends DrivetrainIO {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

    private final SmartSwerveDrivetrain drivetrain;

    public DrivetrainHardware(SchedulerProvider schedulerProvider) {
        super(schedulerProvider.getScheduler());

        SwerveModuleConstantsFactory moduleFactory = new SwerveModuleConstantsFactory()
            .withCouplingGearRatio(0)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorGains(CONSTANTS.getDriveGains())
            .withDriveMotorGearRatio(CONSTANTS.getDriveGearRatio().getReduction())
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withSlipCurrent(75)
            .withSpeedAt12VoltsMps(CONSTANTS.getMaxLinearVelocity().in(Units.MetersPerSecond))
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
            .withSteerMotorGains(CONSTANTS.getSteerGains())
            .withSteerMotorGearRatio(CONSTANTS.getSteerGearRatio().getReduction())
            .withSteerMotorInverted(true)
            .withWheelRadius(CONSTANTS.getWheelDiameter().in(Units.Inches) / 2);

        SwerveModuleConstants frontLeftModule = moduleFactory.createModuleConstants(
            10, 0, 0,
            CONSTANTS.getFrontLeftOffset().getRotations(),
            CONSTANTS.getModuleDistanceFromCentreX().in(Units.Meters),
            CONSTANTS.getModuleDistanceFromCentreY().in(Units.Meters),
            false
        );

        SwerveModuleConstants backLeftModule = moduleFactory.createModuleConstants(
            11, 1, 1,
            CONSTANTS.getBackLeftOffset().getRotations(),
            -CONSTANTS.getModuleDistanceFromCentreX().in(Units.Meters),
            CONSTANTS.getModuleDistanceFromCentreY().in(Units.Meters),
            false
        );

        SwerveModuleConstants backRightModule = moduleFactory.createModuleConstants(
            12, 2, 2,
            CONSTANTS.getBackRightOffset().getRotations(),
            -CONSTANTS.getModuleDistanceFromCentreX().in(Units.Meters),
            -CONSTANTS.getModuleDistanceFromCentreY().in(Units.Meters),
            false
        );

        SwerveModuleConstants frontRightModule = moduleFactory.createModuleConstants(
            13, 3, 3,
            CONSTANTS.getFrontRightOffset().getRotations(),
            CONSTANTS.getModuleDistanceFromCentreX().in(Units.Meters),
            -CONSTANTS.getModuleDistanceFromCentreY().in(Units.Meters),
            false
        );

        drivetrain = new SmartCtreSwerveDrivetrain(
            getScheduler(),
            new SwerveDrivetrainConstants()
                .withCANbusName(CommonConstants.CANIVORE_BUS_NAME)
                .withPigeon2Id(0),
            CONSTANTS.getPigeonConfig(),
            250,
            CONSTANTS.getPoseEstimatorConfig().stateStdDevsMetresMetresRadians(),
            CONSTANTS.getPoseEstimatorConfig().visionStdDevsMetresMetresRadians(),
            frontLeftModule,
            backLeftModule,
            backRightModule,
            frontRightModule
        );
    }

    { registerHandler(Scheduler.EXECUTE, this::addModulesToDashboard); }
    private void addModulesToDashboard() {
        drivetrain.telemetry();
    }

    @Override
    public void updateInputs() {
        inputs.position = drivetrain.getPosition();
        inputs.velocityMetresAndRadiansPerSec = drivetrain.getVelocityMetresAndRadiansPerSec();
    }

    @Override
    public void driveWithVelocity(ChassisSpeeds robotRelativeSpeeds) {
        drivetrain.driveWithVelocity(robotRelativeSpeeds);
    }

    @Override
    public void setCurrentPosition(Pose2d pose) {
        drivetrain.setPosition(pose);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        drivetrain.addVisionMeasurement(pose, timestampSeconds);
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return drivetrain.getKinematics();
    }
}
