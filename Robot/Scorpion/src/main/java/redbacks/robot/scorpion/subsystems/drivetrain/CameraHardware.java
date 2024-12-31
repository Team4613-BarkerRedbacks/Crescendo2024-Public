package redbacks.robot.scorpion.subsystems.drivetrain;

import static redbacks.robot.scorpion.subsystems.drivetrain.ScorpionCameraConstants.*;

import java.util.function.Supplier;

import arachne4.lib.math.geometry.FieldCoordinateSystem.CoordinateConvention;
import arachne4.lib.scheduler.SchedulerProvider;
import redbacks.field.FieldLocations;
import redbacks.lib.hardware.vision.limelight.Limelight3G;
import redbacks.robot.scorpion.Scorpion;

public class CameraHardware extends CameraIO {
    private final Limelight3G centerCamera, leftCamera;
    private final Supplier<CoordinateConvention> coordinateConventionSupplier;

    public CameraHardware(SchedulerProvider schedulerProvider, Scorpion robot) {
        super(schedulerProvider.getScheduler());

        this.coordinateConventionSupplier = robot::getCoordinateConvention;

        centerCamera = new Limelight3G(
            getScheduler(),
            FieldLocations.FIELD_COORDINATE_SYSTEM,
            "limelight-center",
            CENTER_CAMERA_OFFSET_FROM_ROBOT,
            robot::getYawPositionAndVelocity);

        leftCamera = new Limelight3G(
            getScheduler(),
            FieldLocations.FIELD_COORDINATE_SYSTEM,
            "limelight-left",
            LEFT_CAMERA_OFFSET_FROM_ROBOT,
            robot::getYawPositionAndVelocity);
    }

    @Override
    public void updateInputs() {
        inputs.centerMegatag = centerCamera.getMegatagEstimate(coordinateConventionSupplier.get()).orElse(null);
        inputs.centerMegatag2 = centerCamera.getMegatag2Estimate(coordinateConventionSupplier.get()).orElse(null);
        inputs.leftMegatag = leftCamera.getMegatagEstimate(coordinateConventionSupplier.get()).orElse(null);
        inputs.leftMegatag2 = leftCamera.getMegatag2Estimate(coordinateConventionSupplier.get()).orElse(null);
    }
}
