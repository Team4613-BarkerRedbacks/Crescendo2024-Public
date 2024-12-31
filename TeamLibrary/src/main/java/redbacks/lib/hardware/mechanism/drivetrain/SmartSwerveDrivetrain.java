package redbacks.lib.hardware.mechanism.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public interface SmartSwerveDrivetrain {
    default void telemetry() {}

    Pose2d getPosition();

    void setPosition(Pose2d position);
    default void setPosition(Translation2d location, Rotation2d heading) { setPosition(new Pose2d(location, heading)); }
    default void setPosition(Measure<Distance> x, Measure<Distance> y, Rotation2d heading) { setPosition(new Pose2d(x, y, heading)); }

    SwerveDriveKinematics getKinematics();

    void addVisionMeasurement(Pose2d visionCalculatedPositionMetres, double timestampSeconds);

    ChassisSpeeds getVelocityMetresAndRadiansPerSec();
    void driveWithVelocity(ChassisSpeeds robotRelativeSpeeds);

    public static record SwerveDrivePoseEstimatorConfig(Matrix<N3, N1> stateStdDevsMetresMetresRadians, Matrix<N3, N1> visionStdDevsMetresMetresRadians) {}
}
