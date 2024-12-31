package redbacks.robot;

import static redbacks.robot.CommonConstants.*;

import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;
import arachne4.lib.math.geometry.Rotation2dWithCoordinateConvention;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.scheduler.data.DataSource;
import arachne4.lib.subsystems.Subsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.lib.hardware.vision.limelight.Limelight3G.AngularPositionAndVelocity;
import redbacks.robot.CommonScoringTargets.ScoringTarget;

public abstract class Robot2024 extends Subsystem {
    public final DataSource<ScoringTarget> scoringTarget;

    private CoordinateConvention coordinateConvention = COORDINATE_CONVENTIONS.in(Alliance.Red);

    public Robot2024(SchedulerProvider schedulerProvider) {
        super(schedulerProvider);

        this.scoringTarget = new DataSource<>(scheduler, CommonScoringTargets.SPEAKER);

        registerHandler(
            Scheduler.PRE_EXECUTE,
            () -> DriverStation.getAlliance().ifPresent(alliance -> coordinateConvention = COORDINATE_CONVENTIONS.in(alliance)));
    }

    public CoordinateConvention getCoordinateConvention() {
        return coordinateConvention;
    }

    public abstract Pose2d getPosition();
    public abstract ChassisSpeeds getFieldRelativeVelocity();

    public AngularPositionAndVelocity getYawPositionAndVelocity() {
        return new AngularPositionAndVelocity(
            new Rotation2dWithCoordinateConvention(getCoordinateConvention(), getPosition().getRotation()),
            Units.RadiansPerSecond.of(getFieldRelativeVelocity().omegaRadiansPerSecond));
    }
}
