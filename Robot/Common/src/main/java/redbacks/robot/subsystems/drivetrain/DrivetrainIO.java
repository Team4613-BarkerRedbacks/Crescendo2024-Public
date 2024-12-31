package redbacks.robot.subsystems.drivetrain;

import java.util.Collections;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import redbacks.lib.io.InputsProvider;

public abstract class DrivetrainIO extends InputsProvider<DrivetrainInputsAutoLogged> {
    @AutoLog
    public static class DrivetrainInputs {
        public Pose2d position = new Pose2d();
        public ChassisSpeeds velocityMetresAndRadiansPerSec = new ChassisSpeeds();
    }

    public DrivetrainIO(Scheduler scheduler) {
        super(scheduler, DrivetrainConstants.INPUTS_LOGGING_KEY, new DrivetrainInputsAutoLogged());
    }

    public abstract void driveWithVelocity(ChassisSpeeds robotRelativeSpeeds);
    public abstract void setCurrentPosition(Pose2d pose);
    public abstract void addVisionMeasurement(Pose2d pose, double timestampSeconds);
    public abstract SwerveDriveKinematics getKinematics();

    static final Pose2d NULL_LOGGED_POSE = new Pose2d(Double.NaN, Double.NaN, new Rotation2d());
    public void logTargetPosition(Pose2d targetPose) {
        Logger.recordOutput("Drivetrain/Pose/Target", targetPose != null ? targetPose : NULL_LOGGED_POSE);
    }

    static final Trajectory NULL_LOGGED_TRAJECTORY = new Trajectory(Collections.singletonList(new Trajectory.State(0, 0, 0, NULL_LOGGED_POSE, 0)));
    public void logPlannedTrajectory(Trajectory trajectory) {
        Logger.recordOutput("Drivetrain/Trajectory", trajectory != null ? trajectory : NULL_LOGGED_TRAJECTORY);
    }

    public static class Empty extends DrivetrainIO {
        public Empty(Scheduler scheduler) {
            super(scheduler);
        }

        @Override public void updateInputs() {}
        @Override public void driveWithVelocity(ChassisSpeeds robotRelativeSpeeds) {}
        @Override public void setCurrentPosition(Pose2d pose) {}
        @Override public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {}

        @Override public SwerveDriveKinematics getKinematics() {
            return new SwerveDriveKinematics(
                new Translation2d(1, 1),
                new Translation2d(-1, 1),
                new Translation2d(-1, -1),
                new Translation2d(1, -1));
        }
    }
}
