package redbacks.lib.hardware.mechanism.drivetrain.general;

import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import redbacks.lib.hardware.mechanism.drivetrain.SmartSwerveDrivetrain;
import redbacks.lib.hardware.sensor.SmartAngleSensor;

public class GeneralSwerveDrivetrain extends SchedulerProviderBase implements SmartSwerveDrivetrain {
    private final SmartAngleSensor yawSensor;
    private final GeneralSwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;
    public final SwerveDriveKinematics getKinematics() {
        return kinematics;
    } 
    
    private Pose2d currentPosition;
    private ChassisSpeeds currentVelocityMetresAndRadiansPerSec;

    public GeneralSwerveDrivetrain(Scheduler scheduler, SwerveDrivePoseEstimatorConfig poseEstimatorConfig, SmartAngleSensor yawSensor, GeneralSwerveModule... modules) {
        super(scheduler);

        var locations = new Translation2d[modules.length];
        for(int i = 0; i < modules.length; i++) locations[i] = modules[i].getOffsetFromRobotCentre();

        this.yawSensor = yawSensor;
        this.modules = modules.clone();
        this.kinematics = new SwerveDriveKinematics(locations);

        this.odometry = new SwerveDrivePoseEstimator(
            kinematics, 
            yawSensor.getAngle(), 
            getModulePositions(), 
            new Pose2d(), 
            poseEstimatorConfig.stateStdDevsMetresMetresRadians(),
            poseEstimatorConfig.visionStdDevsMetresMetresRadians()
        );
    }

    /* Handlers */

    { registerHandler(Scheduler.PRE_EXECUTE, this::updateOdometry); }
    private void updateOdometry() {
        SwerveModulePosition[] modulePositions = getModulePositions();

        try {
            // If the method call does not throw an exception, update the last known pose
            // We store the pose to reset to in case of errors due to timing in updateWithTime()
            currentPosition = odometry.updateWithTime(Timer.getFPGATimestamp(), yawSensor.getAngle(), modulePositions);
        }
        catch(RuntimeException e) {
            // If updateWithTime() throws an exception, reset to the last known position
            odometry.resetPosition(yawSensor.getAngle(), modulePositions, currentPosition);
        }

        // Rotate robot-relative speeds by the negative of the heading to get field-relative speeds
        currentVelocityMetresAndRadiansPerSec = ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(getModuleStates()),
            currentPosition.getRotation().unaryMinus());
    }

    private boolean hasBeenEnabled = false;
    { registerHandler(Scheduler.GAME_STATE_CHANGE, from(GameState.DISABLED), () -> hasBeenEnabled = true); }
    { registerHandler(Scheduler.INITIALIZE, this::updateModuleMotorAnglesFromSensors); }
    { registerHandler(Scheduler.EXECUTE, GameState.DISABLED, this::updateModuleMotorAnglesFromSensors); }
    private void updateModuleMotorAnglesFromSensors() {
        if (!hasBeenEnabled) {
            for(var module : modules) module.updateMotorAngleFromSensor();
        }
    }

    /* Public interface */

    @Override
    public void driveWithVelocity(ChassisSpeeds velocitiesMetresAndRadiansPerSec) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(velocitiesMetresAndRadiansPerSec);
        for(int i = 0; i < modules.length; i++) modules[i].drive(moduleStates[i]);
    }

    @Override
    public Pose2d getPosition() {
        return currentPosition;
    }

    /**
     * @param posX The position along long end of the field (The centre is 0, your side is negative)
     * @param posY The position along short end of field (The centre is 0, the right is negative)
     * @param angle The heading of the robot (Facing the opposing alliance station is 0)
     */
    @Override
    public void setPosition(Pose2d position) {
        odometry.resetPosition(
            yawSensor.getAngle(),
            getModulePositions(),
            position
        );
        currentPosition = position;
    }

    @Override
    public ChassisSpeeds getVelocityMetresAndRadiansPerSec() {
        return currentVelocityMetresAndRadiansPerSec;
    }

    @Override
    public void addVisionMeasurement(Pose2d visionCalculatedPositionMetres, double timestampSeconds) {
        odometry.addVisionMeasurement(visionCalculatedPositionMetres, timestampSeconds);
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) modulePositions[i] = modules[i].getModulePosition();

        return modulePositions;
    }

    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) moduleStates[i] = modules[i].getModuleState();

        return moduleStates;
    }
}
