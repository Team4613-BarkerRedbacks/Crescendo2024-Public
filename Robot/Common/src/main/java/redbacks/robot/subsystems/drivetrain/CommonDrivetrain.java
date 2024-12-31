package redbacks.robot.subsystems.drivetrain;

import static redbacks.robot.subsystems.drivetrain.HeadingControlUtils.aimAtPoint;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import arachne4.lib.Constants;
import arachne4.lib.behaviours.BehaviourManager;
import arachne4.lib.function.BooleanPredicate;
import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.sequences.Actionable;
import arachne4.lib.sequences.Untilable;
import arachne4.lib.sequences.actions.Action;
import arachne4.lib.sequences.actions.HostAction;
import arachne4.lib.subsystems.Subsystem;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.field.FieldLocations;
import redbacks.robot.CommonConstants;
import redbacks.robot.io.IOProvider;
import redbacks.robot.subsystems.drivetrain.behaviours.AimAtSpeakerDriveBehaviour;
import redbacks.robot.subsystems.drivetrain.behaviours.DrivetrainBehaviour;
import redbacks.robot.subsystems.drivetrain.behaviours.FieldRelativeDriveBehaviour;
import redbacks.robot.subsystems.drivetrain.behaviours.SnapToAngleBehaviour;

public class CommonDrivetrain extends Subsystem {
    private static final DrivetrainConstants CONSTANTS = Constants.get(DrivetrainConstants.class);
    private final CommonDrivetrainMappings mappings = scheduler.getMappings(CommonDrivetrainMappings.class);

    protected final DrivetrainIO io;

    protected final BehaviourManager<DrivetrainBehaviour> behaviour;

    protected final DrivetrainBehaviour fieldRelativeDriveBehaviour;
    protected final DrivetrainBehaviour aimAtSpeakerDriveBehaviour;

    public CommonDrivetrain(SchedulerProvider robot, IOProvider io) {
        super(robot);

        this.io = io.getDrivetrainIO();

        this.fieldRelativeDriveBehaviour = new FieldRelativeDriveBehaviour(io.getDrivetrainIO());
        this.aimAtSpeakerDriveBehaviour = new AimAtSpeakerDriveBehaviour(this, io.getDrivetrainIO());

        this.behaviour = new BehaviourManager<>(robot, fieldRelativeDriveBehaviour);
        registerHandler(Scheduler.EXECUTE, behaviour);
    }

    /* Handlers */

    // Change to driver control by default whenever game state changes
    { registerHandler(Scheduler.GAME_STATE_CHANGE, this::changeToDriverControl); }
    private void changeToDriverControl() {
        behaviour.changeToMode(fieldRelativeDriveBehaviour);
    }

    { registerHandler(Scheduler.EXECUTE, GameState.DRIVER_CONTROLLED_STATES, this::acceptDriverInputs); }
    private void acceptDriverInputs() {
        var inputs = mappings.driverInputs.get();

        behaviour.getCurrentMode().ifPresent((behaviour) -> behaviour.acceptDriverInputs(inputs.vxPercent(), inputs.vyPercent(), inputs.vThetaPercent()));
    }

    { registerHandler(mappings.resetHeading.onPress(), this::resetHeading); }
    private void resetHeading() {
        io.setCurrentPosition(new Pose2d(io.inputs.position.getTranslation(), new Rotation2d(0)));
    }

    { registerHandler(mappings.activateSnapTo0.onChange(), usingToBoolean(this::snapTo0)); }
    private void snapTo0(boolean activate) {
        behaviour.changeToMode(activate ? new SnapToAngleBehaviour(io, Rotation2d.fromDegrees(0)) : fieldRelativeDriveBehaviour);
    }

    { registerHandler(mappings.activateSnapToAmp.onChange(), usingToBoolean(this::snapToAmp));}
    private void snapToAmp(boolean activate) {
        behaviour.changeToMode(activate ? new SnapToAngleBehaviour(io, Rotation2d.fromDegrees(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? 90 : -90)) : fieldRelativeDriveBehaviour);
    }

    { registerHandler(mappings.activateSnapToFeedCentre.onChange(), usingToBoolean(this::snapToFeedCentre));}
    private void snapToFeedCentre(boolean activate) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        var aimAtPoint = aimAtPoint(
            CommonConstants.FEED_TO_CENTER_TARGET_POS.in(alliance),
            false,
            () -> calcFeedYawOffsetFromYVelocityWithSign(CommonConstants.FEED_TO_CENTER_YAW_OFFSET_FROM_Y_VELOCITY));
        behaviour.changeToMode(activate ? new SnapToAngleBehaviour(io, () -> aimAtPoint.apply(getPosition().getTranslation())) : fieldRelativeDriveBehaviour);
    }

    { registerHandler(mappings.activateSnapToFeedAmp.onChange(), usingToBoolean(this::snapToFeedAmp));}
    private void snapToFeedAmp(boolean activate) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        var aimAtPoint = aimAtPoint(
            CommonConstants.FEED_TO_AMP_TARGET_POS.in(alliance),
            false,
            () -> calcFeedYawOffsetFromYVelocityWithSign(CommonConstants.FEED_TO_AMP_YAW_OFFSET_FROM_Y_VELOCITY));
        behaviour.changeToMode(activate ? new SnapToAngleBehaviour(io, () -> aimAtPoint.apply(getPosition().getTranslation())) : fieldRelativeDriveBehaviour);
    }

    private Rotation2d calcFeedYawOffsetFromYVelocityWithSign(InterpolatingTreeMap<Double, Rotation2d> offsetMap) {
        double yVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        Rotation2d absOffset = offsetMap.get(Math.abs(yVelocity));
        return absOffset.times(Math.signum(yVelocity));
    }

    { registerHandler(mappings.aimAtSpeaker.onChange(), usingToBoolean(this::aimAtSpeaker));}
    private void aimAtSpeaker(boolean activate) {
        behaviour.changeToMode(activate ? aimAtSpeakerDriveBehaviour : fieldRelativeDriveBehaviour);
    }

    { registerHandler(Scheduler.EXECUTE, this::displayOdometry); }
    protected void displayOdometry() {
        SmartDashboard.putNumber("Robot X", io.inputs.position.getX());
        SmartDashboard.putNumber("Robot Y", io.inputs.position.getY());
        SmartDashboard.putNumber("Robot Heading", io.inputs.position.getRotation().getDegrees());
        SmartDashboard.putString("Drivetrain Behaviour", behaviour.getCurrentMode().toString());

        var velocity = io.inputs.velocityMetresAndRadiansPerSec;
        SmartDashboard.putNumber("Drive Velocity", Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond));

        Logger.recordOutput("Drivetrain/Pose/Actual", io.inputs.position);
    }

    /* hold */

    public void snapToTargetHeading(Rotation2d targetHeading) {
        behaviour.changeToMode(new SnapToAngleBehaviour(io, targetHeading));
    }

    /* Accessors */

    public Pose2d getPosition() {
        return io.inputs.position;
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return io.inputs.velocityMetresAndRadiansPerSec;
    }

    public Translation2d getOffsetFromSpeaker() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()) return new Translation2d(FieldLocations.EDGE_OF_SUBWOOFER_X.plus(CONSTANTS.getRobotOffsetXFromEdge()), Units.Meters.zero());

        Translation2d position = getPosition().getTranslation();
        return position.minus(new Translation2d(Units.Meters.zero(), FieldLocations.SPEAKER_Y.in(alliance.get())));
    }

    public Translation2d getVelocityOffsetSpeakerPosition() { // Shoot while moving
        Translation2d velocity = new Translation2d(getFieldRelativeVelocity().vxMetersPerSecond, getFieldRelativeVelocity().vyMetersPerSecond);

        Translation2d robotSpeakerOffset = getOffsetFromSpeaker();

        double shotDistance = Math.sqrt(Math.pow(robotSpeakerOffset.getNorm(), 2) + 4);
        double velocityTowardsGoal = robotSpeakerOffset.getNorm() - robotSpeakerOffset.plus(velocity).getNorm();
        double timeToTarget = shotDistance / getAverageNoteVelocity(shotDistance, velocityTowardsGoal);

        SmartDashboard.putNumber("Note time to goal", timeToTarget);
        SmartDashboard.putNumber("velocity towards goal", velocityTowardsGoal);

        return getOffsetFromSpeaker().plus(velocity.times(timeToTarget));
    }

    public double getAverageNoteVelocity(double distanceToGoal, double velocityTowardGoal) {
        double distanceSquaredExponent = -0.04;
        double noteInitalVelocity = 14 + velocityTowardGoal;

        return (((distanceSquaredExponent * Math.pow(distanceToGoal, 3)) * (1d/3d)) + (noteInitalVelocity*distanceToGoal)) / distanceToGoal; 
    }

    public void setPosition(Measure<Distance> posX, Measure<Distance> posY, Rotation2d heading) {
        lastHeadingForMovement = heading;
        io.setCurrentPosition(new Pose2d(posX, posY, heading));
    }

    public void setPosition(Translation2d pos, Rotation2d heading) {
        lastHeadingForMovement = heading;
        io.setCurrentPosition(new Pose2d(pos, heading));
    }

    public void setPosition(Pose2d pose) {
        lastHeadingForMovement = pose.getRotation();
        io.setCurrentPosition(pose);
    }

    public void disableSnapToGoal() {
        behaviour.changeToMode(fieldRelativeDriveBehaviour);
    }


    /* Actionables */

    public Untilable doVelocityToWithControlVectors(ControlVectorList controlVectors, Function<Translation2d, Rotation2d> targetHeadingFromLocation, Measure<Velocity<Distance>> maxVelocity, Measure<Velocity<Distance>> endVelocity) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            Trajectory trajectory;
            double startTimeSeconds;

            Translation2d targetLocation;
            Measure<Velocity<Distance>> endVelocityX;
            Measure<Velocity<Distance>> endVelocityY;

            @Override
            protected void initialize() {
                ProfiledPIDController rotationController = new ProfiledPIDController(CONSTANTS.getRotationalKp(), CONSTANTS.getRotationalKi(), CONSTANTS.getRotationalKd(), CONSTANTS.getAutoRotationMotionConstraints());
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                    new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                    rotationController
                );
                
                var startVelocityMetresPerSec = Math.hypot(io.inputs.velocityMetresAndRadiansPerSec.vxMetersPerSecond, io.inputs.velocityMetresAndRadiansPerSec.vyMetersPerSecond);
                
                var lastVector = controlVectors.get(controlVectors.size() - 1);
                var endAngleRadians = Math.atan2(lastVector.y[1], lastVector.x[1]);
                
                targetLocation = new Translation2d(lastVector.x[0], lastVector.y[0]);
                endVelocityX = endVelocity.times(Math.cos(endAngleRadians));
                endVelocityY = endVelocity.times(Math.sin(endAngleRadians));

                TrajectoryConfig trajectoryconfig = createTrajectoryConfig(maxVelocity)
                    .setStartVelocity(startVelocityMetresPerSec)
                    .setEndVelocity(endVelocity);

                trajectory = TrajectoryGenerator.generateTrajectory(
                    controlVectors,
                    trajectoryconfig
                );
        
                startTimeSeconds = Timer.getFPGATimestamp();

                SmartDashboard.putNumberArray("Drivetrain Target", new double[] { lastVector.x[0], lastVector.y[0] });
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = io.inputs.position;

                io.driveWithVelocity(controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTimeSeconds),
                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                ));
            }

			@Override
			protected void end() {
				io.driveWithVelocity(new ChassisSpeeds(endVelocityX.in(Units.MetersPerSecond), endVelocityY.in(Units.MetersPerSecond), 0));
			}

			@Override
			protected boolean isFinished() {
                Pose2d currentPosition = io.inputs.position;
                double timeElapsedSeconds = Timer.getFPGATimestamp() - startTimeSeconds;

				return targetLocation.minus(currentPosition.getTranslation()).getNorm() < CONSTANTS.getPositionToleranceMetres()
                    && Math.abs(targetHeadingFromLocation.apply(targetLocation).minus(currentPosition.getRotation()).getRadians()) < CONSTANTS.getRotationToleranceRadians()
                    && timeElapsedSeconds >= trajectory.getTotalTimeSeconds();
			}
        };
    }

    public DoSplineToConfig doMoveToPose(Pose2d target) {
        return doMoveTo(target.getTranslation()).withHeading(target.getRotation());
    }

    public DoSplineToConfig doMoveTo(Translation2d target) {
        return new DoSplineToConfig(target);
    }

    private Translation2d lastSplineTarget;
    private Measure<Velocity<Distance>> lastSplineEndVelocity;
    private Rotation2d lastHeadingForMovement = Rotation2d.fromDegrees(0);

    public final class DoSplineToConfig implements Untilable {
        private final Translation2d target;
        private final Function<Translation2d, Rotation2d> targetHeadingFromLocation;

        private final Measure<Velocity<Distance>> endVelocity;
        private final Measure<Velocity<Distance>> maxVelocity;
        private final List<Translation2d> waypoints;

        private final Translation2d startPositionForPrecomputation;
        private final Measure<Velocity<Distance>> startVelocityForPrecomputation;
        private final Trajectory precomputedTrajectory;

        private final Translation2d lastSplineTargetBeforeBuilder;
        private final Measure<Velocity<Distance>> lastSplineEndVelocityBeforeBuilder;

        private DoSplineToConfig(Translation2d target) {
            this.target = target;
            targetHeadingFromLocation = (pos) -> lastHeadingForMovement;
            endVelocity = Units.MetersPerSecond.zero();
            maxVelocity = CONSTANTS.getMaxLinearVelocity();
            waypoints = Collections.emptyList();
            startPositionForPrecomputation = null;
            startVelocityForPrecomputation = null;
            precomputedTrajectory = null;

            this.lastSplineTargetBeforeBuilder = CommonDrivetrain.this.lastSplineTarget;
            this.lastSplineEndVelocityBeforeBuilder = CommonDrivetrain.this.lastSplineEndVelocity;
            CommonDrivetrain.this.lastSplineTarget = target;
            CommonDrivetrain.this.lastSplineEndVelocity = endVelocity;
        }

        private DoSplineToConfig(
                Translation2d target, Function<Translation2d, Rotation2d> targetHeadingFromLocation,
                Measure<Velocity<Distance>> endVelocity, Measure<Velocity<Distance>> maxVelocity,
                List<Translation2d> waypoints,
                Translation2d startPositionForPrecomputation, Measure<Velocity<Distance>> startVelocityForPrecomputation,
                Translation2d lastSplineTargetBeforeBuilder, Measure<Velocity<Distance>> lastSplineEndVelocityBeforeBuilder) {
            this.target = target;
            this.targetHeadingFromLocation = targetHeadingFromLocation;
            this.endVelocity = endVelocity;
            this.maxVelocity = maxVelocity;
            this.waypoints = waypoints;
            this.startPositionForPrecomputation = startPositionForPrecomputation;
            this.startVelocityForPrecomputation = startVelocityForPrecomputation;
            precomputedTrajectory = computeTrajectory(startPositionForPrecomputation, startVelocityForPrecomputation);

            this.lastSplineTargetBeforeBuilder = lastSplineTargetBeforeBuilder;
            this.lastSplineEndVelocityBeforeBuilder = lastSplineEndVelocityBeforeBuilder;
        }

        private Trajectory computeTrajectory(Translation2d startPosition, Measure<Velocity<Distance>> startVelocity) {
            if (startPosition == null || startVelocity == null) return null;

            Rotation2d startDirection = (waypoints.isEmpty() ? target : waypoints.get(0)).minus(startPosition).getAngle();
            Rotation2d endDirection = target.minus(waypoints.isEmpty() ? startPosition : waypoints.get(waypoints.size() - 1)).getAngle();

            TrajectoryConfig trajectoryconfig = createTrajectoryConfig(maxVelocity)
                .setStartVelocity(startVelocity)
                .setEndVelocity(endVelocity);

            return TrajectoryGenerator.generateTrajectory(
                new Pose2d(startPosition, startDirection),
                waypoints,
                new Pose2d(target, endDirection),
                trajectoryconfig
            );
        }

        public DoSplineToConfig withHeading(Rotation2d targetHeading) {
            return new DoSplineToConfig(target, (pos) -> targetHeading, endVelocity, maxVelocity, waypoints, startPositionForPrecomputation, startVelocityForPrecomputation, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public DoSplineToConfig withHeading(Function<Translation2d, Rotation2d> targetHeadingFromLocation) {
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, waypoints, startPositionForPrecomputation, startVelocityForPrecomputation, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public DoSplineToConfig withEndVelocity(Measure<Velocity<Distance>> endVelocity) {
            CommonDrivetrain.this.lastSplineEndVelocity = endVelocity;
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, waypoints, startPositionForPrecomputation, startVelocityForPrecomputation, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public DoSplineToConfig withMaxVelocity(Measure<Velocity<Distance>> maxVelocity) {
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, waypoints, startPositionForPrecomputation, startVelocityForPrecomputation, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public DoSplineToConfig withWaypoints(List<Translation2d> waypoints) {
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, waypoints, startPositionForPrecomputation, startVelocityForPrecomputation, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public DoSplineToConfig withWaypoints(Translation2d... waypoints) {
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, Arrays.asList(waypoints), startPositionForPrecomputation, startVelocityForPrecomputation, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public Untilable precomputedWithStartPosition(Translation2d startPosition) {
            return precomputedWithStartPositionAndVelocity(startPosition, Units.MetersPerSecond.zero());
        }

        public Untilable precomputedWithStartPositionAndVelocity(Translation2d startPosition, Measure<Velocity<Distance>> startVelocity) {
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, waypoints, startPosition, startVelocity, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        public Untilable precomputedFromLastTarget() {
            return new DoSplineToConfig(target, targetHeadingFromLocation, endVelocity, maxVelocity, waypoints, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder, lastSplineTargetBeforeBuilder, lastSplineEndVelocityBeforeBuilder);
        }

        @Override
        public Action asAction(HostAction host, BooleanPredicate conditionModifier) {
            return new Action(host, conditionModifier) {
                HolonomicDriveController controller;
                Trajectory trajectory;
                double startTimeSeconds;
                Measure<Velocity<Distance>> endVelocityX;
                Measure<Velocity<Distance>> endVelocityY;
    
                @Override
                protected void initialize() {
                    ProfiledPIDController rotationController = new ProfiledPIDController(CONSTANTS.getRotationalKp(), CONSTANTS.getRotationalKi(), CONSTANTS.getRotationalKd(), CONSTANTS.getAutoRotationMotionConstraints());
                    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    
                    controller = new HolonomicDriveController(
                        new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                        new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                        rotationController
                    );

                    if (precomputedTrajectory != null) {
                        trajectory = precomputedTrajectory;
                    }
                    else {
                        Translation2d currentLocation = io.inputs.position.getTranslation();

                        ChassisSpeeds chassisSpeeds = io.inputs.velocityMetresAndRadiansPerSec;
                        Measure<Velocity<Distance>> startVelocity = Units.MetersPerSecond.of(Math.sqrt(
                            Math.pow(chassisSpeeds.vxMetersPerSecond, 2) +
                            Math.pow(chassisSpeeds.vyMetersPerSecond, 2)));

                        trajectory = computeTrajectory(currentLocation, startVelocity);
                    }

                    Rotation2d deltaDirection = target.minus(trajectory.getInitialPose().getTranslation()).getAngle();
                    endVelocityX = endVelocity.times(deltaDirection.getCos());
                    endVelocityY = endVelocity.times(deltaDirection.getSin());

                    startTimeSeconds = Timer.getFPGATimestamp();

                    io.logPlannedTrajectory(trajectory);
                }

                @Override
                protected void execute() {
                    Pose2d currentPosition = io.inputs.position;
                    lastHeadingForMovement = targetHeadingFromLocation.apply(currentPosition.getTranslation());
    
                    Trajectory.State desiredState = trajectory.sample(Timer.getFPGATimestamp() - startTimeSeconds);
                    io.logTargetPosition(new Pose2d(desiredState.poseMeters.getTranslation(), targetHeadingFromLocation.apply(desiredState.poseMeters.getTranslation())));

                    io.driveWithVelocity(controller.calculate(currentPosition, desiredState, lastHeadingForMovement));
                }
    
                @Override
                protected void end() {
                    lastHeadingForMovement = targetHeadingFromLocation.apply(target);
                    io.driveWithVelocity(new ChassisSpeeds(endVelocityX, endVelocityY, Units.RadiansPerSecond.zero()));

                    io.logPlannedTrajectory(null);
                    io.logTargetPosition(null);
                }
    
                @Override
                protected boolean isFinished() {
                    double timeElapsedSeconds = Timer.getFPGATimestamp() - startTimeSeconds;

                    if (endVelocity.gt(Units.MetersPerSecond.zero())) {
                        return timeElapsedSeconds >= trajectory.getTotalTimeSeconds();
                    }
                    else {
                        Pose2d currentPosition = io.inputs.position;

                        return target.minus(currentPosition.getTranslation()).getNorm() < CONSTANTS.getPositionToleranceMetres()
                            && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < CONSTANTS.getRotationToleranceRadians()
                            && timeElapsedSeconds >= trajectory.getTotalTimeSeconds();
                    }
                }
            };
        }
    }

    public Untilable doMoveToWithLerpedRotation(Translation2d target, Rotation2d targetAngle, boolean rotateAntiClockwise) {
        return doMoveToWithLerpedRotation(target, targetAngle, CONSTANTS.getMaxLinearVelocity(), rotateAntiClockwise);
    }

    public Untilable doMoveToWithLerpedRotation(Translation2d target, Rotation2d targetAngle, Measure<Velocity<Distance>> maxVelocity, boolean rotateAntiClockwise) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            Trajectory trajectory;
            Function<Translation2d, Rotation2d> targetHeadingFromLocation;
            double startTimeSeconds;
            double startVelocity;

            @Override
            protected void initialize() {
                controller = new HolonomicDriveController(
                    new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                    new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                    new ProfiledPIDController(CONSTANTS.getRotationalKp(), CONSTANTS.getRotationalKi(), CONSTANTS.getRotationalKd(), CONSTANTS.getAutoRotationMotionConstraints())
                );

                startVelocity = Math.hypot(io.inputs.velocityMetresAndRadiansPerSec.vxMetersPerSecond, io.inputs.velocityMetresAndRadiansPerSec.vyMetersPerSecond);
                var robotPose = io.inputs.position;
                Translation2d initialLocation = robotPose.getTranslation();
                Translation2d targetLocation = target;
                Translation2d delta = target.minus(initialLocation);
                Rotation2d deltaDirection = new Rotation2d(delta.getX(), delta.getY());
                TrajectoryConfig trajectoryconfig = createTrajectoryConfig(maxVelocity)
                    .setStartVelocity(startVelocity);

                trajectory = TrajectoryGenerator.generateTrajectory(
                    new Pose2d(initialLocation, deltaDirection),
                    Collections.emptyList(),
                    new Pose2d(targetLocation, deltaDirection),
                    trajectoryconfig
                );

                startTimeSeconds = Timer.getFPGATimestamp();

                // Lerping logic
                var initialAngleRadians = robotPose.getRotation().getRadians();
                var workingFinalAngleRadians = targetAngle.getRadians();

                if(rotateAntiClockwise) {
                    while(workingFinalAngleRadians < initialAngleRadians) workingFinalAngleRadians += 2 * Math.PI;
                }
                else {
                    while(workingFinalAngleRadians > initialAngleRadians) workingFinalAngleRadians -= 2 * Math.PI;
                }

                final var finalAngleRadians = workingFinalAngleRadians;

                targetHeadingFromLocation = (position) -> {
                    var distanceFromStart = position.minus(initialLocation).getNorm();
                    var distanceFromEnd = position.minus(targetLocation).getNorm();

                    var percentageOfTravel = Math.min(distanceFromStart / (distanceFromStart + distanceFromEnd) + CONSTANTS.getRotationLerpLeadPercentage(), 1);

                    return Rotation2d.fromRadians(initialAngleRadians * (1 - percentageOfTravel) + finalAngleRadians * percentageOfTravel);
                };

                SmartDashboard.putNumberArray("Drivetrain Target", new double[] { targetLocation.getX(), targetLocation.getY() });
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = io.inputs.position;

                io.driveWithVelocity(controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTimeSeconds),

                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                ));
            }

			@Override
			protected void end() {
				io.driveWithVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                Pose2d currentPosition = io.inputs.position;
                double timeElapsedSeconds = Timer.getFPGATimestamp() - startTimeSeconds;

				return target.minus(currentPosition.getTranslation()).getNorm() < CONSTANTS.getPositionToleranceMetres()
                    && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < CONSTANTS.getRotationToleranceRadians()
                    && timeElapsedSeconds >= trajectory.getTotalTimeSeconds();
			}
        };
    }

    public Untilable doPathTo(Trajectory trajectory, Function<Translation2d, Rotation2d> targetHeadingFromLocation) {
        return (host, conditionModifier) -> new Action(host, conditionModifier) {
            HolonomicDriveController controller;
            double startTimeSeconds;
            double endVelocityMetresPerSec;
            double endVelocityXMetresPerSec, endVelocityYMetresPerSec;

            @Override
            protected void initialize() {
                ProfiledPIDController rotationController = new ProfiledPIDController(CONSTANTS.getRotationalKp(), CONSTANTS.getRotationalKi(), CONSTANTS.getRotationalKd(), CONSTANTS.getAutoRotationMotionConstraints());
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                controller = new HolonomicDriveController(
                    new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                    new PIDController(CONSTANTS.getLinearKp(), CONSTANTS.getLinearKi(), CONSTANTS.getLinearKd()),
                    rotationController
                );

                var states = trajectory.getStates();
                var lastState = states.get(states.size() - 1);
                double endAngleRadians = lastState.poseMeters.getRotation().getRadians();
                endVelocityMetresPerSec = lastState.velocityMetersPerSecond;
                endVelocityXMetresPerSec = Math.cos(endAngleRadians) * endVelocityMetresPerSec;
                endVelocityYMetresPerSec = Math.sin(endAngleRadians) * endVelocityMetresPerSec;

                startTimeSeconds = Timer.getFPGATimestamp();
            }

            @Override
            protected void execute() {
                Pose2d currentPosition = io.inputs.position;

                io.driveWithVelocity(controller.calculate(
                    currentPosition,
                    trajectory.sample(Timer.getFPGATimestamp() - startTimeSeconds),
                    targetHeadingFromLocation.apply(currentPosition.getTranslation())
                ));
            }

			@Override
			protected void end() {
				io.driveWithVelocity(new ChassisSpeeds(endVelocityXMetresPerSec, endVelocityYMetresPerSec, 0));
			}

			@Override
			protected boolean isFinished() {
                Pose2d currentPosition = io.inputs.position;
                double timeElapsedSeconds = Timer.getFPGATimestamp() - startTimeSeconds;
                Translation2d target = trajectory.sample(timeElapsedSeconds).poseMeters.getTranslation();

				return target.minus(currentPosition.getTranslation()).getNorm() < CONSTANTS.getPositionToleranceMetres() * (endVelocityMetresPerSec == 0 ? 1 : 2)
                    && Math.abs(targetHeadingFromLocation.apply(target).minus(currentPosition.getRotation()).getRadians()) < CONSTANTS.getRotationToleranceRadians()
                    && timeElapsedSeconds >= trajectory.getTotalTimeSeconds();
			}
        };
    }

    public Actionable doTurnTo(Rotation2d targetAngle) {
		return (host) -> new Action(host) {
            ProfiledPIDController controller;

            @Override
            protected void initialize() {
                controller = new ProfiledPIDController(CONSTANTS.getRotationalKp(), CONSTANTS.getRotationalKi(), CONSTANTS.getRotationalKd(), CONSTANTS.getAutoRotationMotionConstraints());
                controller.enableContinuousInput(-Math.PI, Math.PI);
                controller.reset(io.inputs.position.getRotation().getRadians());
                controller.setGoal(targetAngle.getRadians());
                controller.setTolerance(CONSTANTS.getRotationToleranceRadians());
            }

            @Override
            protected void execute() {
                io.driveWithVelocity(new ChassisSpeeds(
                    0,
                    0,
                    controller.calculate(io.inputs.position.getRotation().getRadians())
                ));
            }

			@Override
			protected void end() {
				io.driveWithVelocity(new ChassisSpeeds(0, 0, 0));
			}

			@Override
			protected boolean isFinished() {
                return controller.atGoal();
			}
        };
	}

    public TrajectoryConfig createTrajectoryConfig(Measure<Velocity<Distance>> maxVelocity) {
        return new TrajectoryConfig(maxVelocity, CONSTANTS.getMaxAcceleration())
            .setKinematics(io.getKinematics());
    }

    public TrajectoryConfig createTrajectoryConfig() {
        return createTrajectoryConfig(CONSTANTS.getMaxLinearVelocity());
    }
}