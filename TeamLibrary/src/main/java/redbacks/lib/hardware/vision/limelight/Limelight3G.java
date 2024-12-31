package redbacks.lib.hardware.vision.limelight;

import java.util.Optional;
import java.util.function.Supplier;

import arachne4.lib.game.GameState;
import arachne4.lib.math.ArachneMath;
import arachne4.lib.math.geometry.FieldCoordinateSystem;
import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;
import arachne4.lib.math.geometry.Rotation2dWithCoordinateConvention;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import io.limelightvision.LimelightHelpers;
import io.limelightvision.LimelightHelpers.PoseEstimate;

public class Limelight3G extends SchedulerProviderBase {
    private static final CoordinateConvention LIMELIGHT_INTERNAL_COORDINATE_CONVENTION = FieldCoordinateSystem.BLUE_RIGHT_ORIGIN;

    private final FieldCoordinateSystem coordinateSystem;

    private final String cameraName;
    private final NetworkTable table;

    private Optional<Supplier<Translation2d>> robotToMovingCameraOffset = Optional.empty();

    private double lastAngleDegrees = 0;

    public Limelight3G(
            Scheduler scheduler, FieldCoordinateSystem coordinateSystem,
            String cameraName, Pose3d fixedRobotToCameraOffset, Supplier<AngularPositionAndVelocity> orientationSupplier) {
        super(scheduler);

        this.coordinateSystem = coordinateSystem;

        this.cameraName = cameraName;
        this.table = NetworkTableInstance.getDefault().getTable(cameraName);

        final double[] cameraPoseArray = LimelightHelpers.pose3dToArray(fixedRobotToCameraOffset);
        registerHandler(Scheduler.EXECUTE, GameState.DISABLED, () -> table.getEntry("camerapose_robotspace_set").setDoubleArray(cameraPoseArray));

        registerHandler(Scheduler.EXECUTE, () -> {
            AngularPositionAndVelocity yawAndRotationalVelocity = orientationSupplier.get();

            double angleDegrees = ArachneMath.getClosestEquivalentAngle(
                yawAndRotationalVelocity.angle.getIn(LIMELIGHT_INTERNAL_COORDINATE_CONVENTION).getDegrees(),
                lastAngleDegrees,
                Units.Degrees);

            LimelightHelpers.SetRobotOrientation(
                cameraName,
                angleDegrees, yawAndRotationalVelocity.velocity.in(Units.DegreesPerSecond),
                0, 0,
                0, 0);

            lastAngleDegrees = angleDegrees;
        });
    }

    public Limelight3G(
            Scheduler scheduler, FieldCoordinateSystem coordinateSystem,
            String cameraName, HeightRollPitch heightRollPitch, Supplier<Translation2d> robotToCameraOffsetSupplier, Supplier<AngularPositionAndVelocity> orientationSupplier) {
        this(
            scheduler,
            coordinateSystem,
            cameraName,
            new Pose3d(
                0, 0, heightRollPitch.height.in(Units.Meters),
                new Rotation3d(heightRollPitch.roll.getRadians(), heightRollPitch.pitch.getRadians(), 0)),
            orientationSupplier);

        this.robotToMovingCameraOffset = Optional.of(robotToCameraOffsetSupplier);
    }

    public Optional<MegatagData> getMegatagEstimate(CoordinateConvention coordinateConvention) {
        return convertPoseEstimateToMegatagData(LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName), coordinateConvention);
    }

    public Optional<MegatagData> getMegatag2Estimate(CoordinateConvention coordinateConvention) {
        return convertPoseEstimateToMegatagData(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName), coordinateConvention);
    }

    private Optional<MegatagData> convertPoseEstimateToMegatagData(PoseEstimate poseEstimate, CoordinateConvention coordinateConvention) {
        if (poseEstimate.tagCount == 0) return Optional.empty();

        int[] tagIds = new int[poseEstimate.tagCount];
        for (int i = 0; i < tagIds.length; i++) tagIds[i] = poseEstimate.rawFiducials[i].id;

        Pose2d pose = poseEstimate.pose;
        if (robotToMovingCameraOffset.isPresent()) {
            Translation2d robotToCamera = robotToMovingCameraOffset.get().get();
            pose = new Pose2d(pose.getTranslation().plus(robotToCamera), pose.getRotation());
        }

        return Optional.of(new MegatagData(
            coordinateSystem.convert(pose, LIMELIGHT_INTERNAL_COORDINATE_CONVENTION, coordinateConvention),
            poseEstimate.timestampSeconds,
            poseEstimate.latency,
            poseEstimate.avgTagDist,
            tagIds));
    }

    public static final record HeightRollPitch(Measure<Distance> height, Rotation2d roll, Rotation2d pitch) {}
    public static final record AngularPositionAndVelocity(Rotation2dWithCoordinateConvention angle, Measure<Velocity<Angle>> velocity) {
        public AngularPositionAndVelocity plus(AngularPositionAndVelocity other) {
            Rotation2d thisAngle = this.angle.getIn(LIMELIGHT_INTERNAL_COORDINATE_CONVENTION);
            Rotation2d otherAngle = other.angle.getIn(LIMELIGHT_INTERNAL_COORDINATE_CONVENTION);

            return new AngularPositionAndVelocity(
                new Rotation2dWithCoordinateConvention(LIMELIGHT_INTERNAL_COORDINATE_CONVENTION, thisAngle.plus(otherAngle)),
                this.velocity.plus(other.velocity));
        }
    }
}
