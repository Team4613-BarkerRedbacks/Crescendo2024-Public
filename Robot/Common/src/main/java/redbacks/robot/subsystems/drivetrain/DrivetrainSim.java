package redbacks.robot.subsystems.drivetrain;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import arachne4.lib.Constants;
import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import redbacks.lib.math.characteristics.PidConfig;
import redbacks.lib.math.characteristics.PidConfig.ContinuousInputBounds;
import redbacks.lib.math.characteristics.PidfConfig;
import redbacks.lib.simulation.MotorSimConfig;
import redbacks.lib.simulation.mechanism.SmartGenericMotorSim;
import redbacks.robot.CommonConstants;

public class DrivetrainSim {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);

    public static class WithPerfectVelocity extends DrivetrainIO {
        private Pose2d pose = new Pose2d();
        private ChassisSpeeds robotRelativeVelocity = new ChassisSpeeds();

        private final SwerveDriveKinematics kinematics;

        public WithPerfectVelocity(Scheduler scheduler) {
            super(scheduler);

            Measure<Distance> moduleDx = CONSTANTS.getModuleDistanceFromCentreX();
            Measure<Distance> moduleDy = CONSTANTS.getModuleDistanceFromCentreY();

            this.kinematics = new SwerveDriveKinematics(
                new Translation2d(moduleDx, moduleDy),
                new Translation2d(moduleDx.negate(), moduleDy),
                new Translation2d(moduleDx.negate(), moduleDy.negate()),
                new Translation2d(moduleDx, moduleDy.negate()));
        }

        @Override
        public void updateInputs() {
            var dtSeconds = CommonConstants.LOOP_PERIOD.in(Units.Seconds);

            var moduleStates = kinematics.toSwerveModuleStates(robotRelativeVelocity);
            var twist = kinematics.toTwist2d(Arrays.stream(moduleStates)
                .map(state -> new SwerveModulePosition(state.speedMetersPerSecond * dtSeconds, state.angle))
                .toArray(SwerveModulePosition[]::new));
            pose = pose.exp(twist);

            inputs.position = pose;
            inputs.velocityMetresAndRadiansPerSec = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, pose.getRotation());

            Logger.recordOutput("Drivetrain pose", pose);
        }
    
        @Override
        public void driveWithVelocity(ChassisSpeeds robotRelativeSpeeds) {
            this.robotRelativeVelocity = robotRelativeSpeeds;
        }
    
        @Override
        public void setCurrentPosition(Pose2d pose) {
            this.pose = pose;
        }
    
        @Override
        public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        }
    
        @Override
        public SwerveDriveKinematics getKinematics() {
            return kinematics;
        }
    }

    public static class WithFullSimulation extends DrivetrainIO {
        private static final double AVAILABLE_VOLTAGE = 12.7;

        private final SwerveDrivePoseEstimator poseEstimator;
        private final SwerveDriveKinematics kinematics;
        private final List<ModuleSim> moduleSims;

        public WithFullSimulation(Scheduler scheduler) {
            super(scheduler);

            Measure<Distance> moduleDx = CONSTANTS.getModuleDistanceFromCentreX();
            Measure<Distance> moduleDy = CONSTANTS.getModuleDistanceFromCentreY();

            // We use FL, FR, BL, BR order here because AdvantageScope displays our normal order incorrectly
            var modulePositions = new Translation2d[] {
                new Translation2d(moduleDx, moduleDy),
                new Translation2d(moduleDx, moduleDy.negate()),
                new Translation2d(moduleDx.negate(), moduleDy),
                new Translation2d(moduleDx.negate(), moduleDy.negate())
            };

            this.kinematics = new SwerveDriveKinematics(modulePositions);
            this.moduleSims = Arrays.stream(modulePositions)
                .map(position -> new ModuleSim())
                .toList();

            this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                new Rotation2d(),
                moduleSims.stream()
                    .map(ModuleSim::getPosition)
                    .toArray(SwerveModulePosition[]::new),
                new Pose2d(),
                CONSTANTS.getPoseEstimatorConfig().stateStdDevsMetresMetresRadians(),
                CONSTANTS.getPoseEstimatorConfig().visionStdDevsMetresMetresRadians());
        }

        @Override
        public void updateInputs() {
            // Previous and current module positions
            var previousModulePositions = new SwerveDriveWheelPositions(moduleSims.stream()
                .map(ModuleSim::getPosition)
                .toArray(SwerveModulePosition[]::new));

            moduleSims.forEach(sim -> sim.update(CommonConstants.LOOP_PERIOD));
            var modulePositions = new SwerveDriveWheelPositions(moduleSims.stream()
                .map(ModuleSim::getPosition)
                .toArray(SwerveModulePosition[]::new));

            // Gyro calculations
            var priorPose = poseEstimator.getEstimatedPosition();
            var twist = kinematics.toTwist2d(previousModulePositions, modulePositions);

            poseEstimator.update(priorPose.exp(twist).getRotation(), modulePositions);

            inputs.position = poseEstimator.getEstimatedPosition();
            inputs.velocityMetresAndRadiansPerSec = ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(moduleSims.stream()
                    .map(ModuleSim::getState)
                    .toArray(SwerveModuleState[]::new)),
                inputs.position.getRotation());

            Logger.recordOutput("Drivetrain/Modules/Actual", moduleSims.stream()
                .map(ModuleSim::getState)
                .toArray(SwerveModuleState[]::new));
        }

        @Override
        public void driveWithVelocity(ChassisSpeeds robotRelativeSpeeds) {
            var moduleTargetStates = kinematics.toSwerveModuleStates(ChassisSpeeds.discretize(robotRelativeSpeeds, CommonConstants.LOOP_PERIOD.in(Units.Seconds)));

            var optimisedStates = new SwerveModuleState[moduleTargetStates.length];
            for (int i = 0; i < moduleTargetStates.length; i++) {
                optimisedStates[i] = moduleSims.get(i).setTargetState(moduleTargetStates[i]);
            }

            Logger.recordOutput("Drivetrain/Modules/Target", optimisedStates);
        }

        @Override
        public void setCurrentPosition(Pose2d pose) {
            poseEstimator.resetPosition(
                pose.getRotation(),
                moduleSims.stream()
                    .map(ModuleSim::getPosition)
                    .toArray(SwerveModulePosition[]::new),
                pose);
        }

        @Override
        public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
            poseEstimator.addVisionMeasurement(pose, timestampSeconds);
        }

        @Override
        public SwerveDriveKinematics getKinematics() {
            return kinematics;
        }

        public Pose2d getCurrentPose() {
            return poseEstimator.getEstimatedPosition();
        }

        private static class ModuleSim {
            private final SmartGenericMotorSim driveSim, steerSim;
            private final Measure<Distance> wheelRadius;

            private ModuleSim() {
                this.wheelRadius = CONSTANTS.getWheelDiameter().divide(2);

                this.driveSim = new SmartGenericMotorSim(
                    CommonConstants.LOOP_PERIOD,
                    new SmartGenericMotorSim.Characteristics(
                        new MotorSimConfig(DCMotor.getKrakenX60Foc(1), CONSTANTS.getDriveGearRatio(), 0.025 /* Guessed */),
                        null,
                        null,
                        new PidfConfig(0.5, 0, 0, 12 * wheelRadius.in(Units.Meters) / CONSTANTS.getMaxLinearVelocity().in(Units.MetersPerSecond)),
                        () -> AVAILABLE_VOLTAGE));

                this.steerSim = new SmartGenericMotorSim(
                    CommonConstants.LOOP_PERIOD,
                    new SmartGenericMotorSim.Characteristics(
                        new MotorSimConfig(DCMotor.getFalcon500Foc(1), CONSTANTS.getSteerGearRatio(), 0.004 /* Guessed */),
                        null,
                        new PidConfig(50, 0, 0, new ContinuousInputBounds(0, 2 * Math.PI)),
                        null,
                        () -> 12));
            }

            public void update(Measure<Time> timeSinceLastUpdate) {
                driveSim.update();
                steerSim.update();
            }

            /**
             * Sets the unoptimised target state (speed and angle) for the module
             * 
             * @param targetState The target state (speed and angle)
             * @return The optimised target state that the module will actually target
             */
            public SwerveModuleState setTargetState(SwerveModuleState targetState) {
                var optimisedTarget = SwerveModuleState.optimize(targetState, Rotation2d.fromRadians(steerSim.getCurrentAngle().getRadians()));

                driveSim.setTargetVelocity(Units.RadiansPerSecond.of(optimisedTarget.speedMetersPerSecond / wheelRadius.in(Units.Meters)));
                steerSim.setTargetAngle(optimisedTarget.angle);

                return optimisedTarget;
            }

            public SwerveModuleState getState() {
                return new SwerveModuleState(
                    driveSim.getCurrentVelocity().in(Units.RadiansPerSecond) * wheelRadius.in(Units.Meters),
                    steerSim.getCurrentAngle());
            }

            public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(
                    driveSim.getCurrentAngle().getRadians() * wheelRadius.in(Units.Meters),
                    steerSim.getCurrentAngle());
            }
        }

        public ChassisSpeeds getFieldRelativeVelocity() {
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(moduleSims.stream()
                    .map(ModuleSim::getState)
                    .toArray(SwerveModuleState[]::new)),
                getCurrentPose().getRotation());
        }
    }
}
