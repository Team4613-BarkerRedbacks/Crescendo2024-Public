package redbacks.robot.scorpion.subsystems.drivetrain;

import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.lib.hardware.vision.limelight.MegatagData;
import redbacks.robot.scorpion.io.ScorpionIOProvider;
import redbacks.robot.scorpion.subsystems.drivetrain.behaviours.SnapToTrapBehaviour;
import redbacks.robot.subsystems.drivetrain.CommonDrivetrain;
import redbacks.robot.subsystems.drivetrain.behaviours.DrivetrainBehaviour;

public class ScorpionDrivetrain extends CommonDrivetrain {
    private final ScorpionDrivetrainMappings mappings = scheduler.getMappings(ScorpionDrivetrainMappings.class);

    private final CameraIO cameraIO;

    private final DrivetrainBehaviour snapToTrapDriveBehaviour;

    public ScorpionDrivetrain(SchedulerProvider robot, ScorpionIOProvider io) {
        super(robot, io);

        this.cameraIO = io.getCameraIO();

        this.snapToTrapDriveBehaviour = new SnapToTrapBehaviour(io.getDrivetrainIO());
    }

    @Override
    protected void displayOdometry() {
        super.displayOdometry();
        SmartDashboard.putBoolean("Center camera sees AprilTag", cameraIO.inputs.centerMegatag2.visibleTags().length > 0);
        SmartDashboard.putBoolean("Left camera sees AprilTag", cameraIO.inputs.leftMegatag2.visibleTags().length > 0);
    }

    { registerHandler(Scheduler.EXECUTE, GameState.DRIVER_CONTROLLED_STATES, this::updateOdometryFromVision); }
    private void updateOdometryFromVision() {
        updateOdometryFromSingleCamera(cameraIO.inputs.centerMegatag2);
        updateOdometryFromSingleCamera(cameraIO.inputs.leftMegatag2);
    }

    private void updateOdometryFromSingleCamera(MegatagData data) {
        if (data.visibleTags().length == 0) return;

        io.addVisionMeasurement(data.fieldRelativePose(), data.timestampSeconds());
    }

    { registerHandler(mappings.activateSnapToTrap.onChange(), usingToBoolean(this::snapToTrap));}
    private void snapToTrap(boolean activate) {
        behaviour.changeToMode(activate ? snapToTrapDriveBehaviour : fieldRelativeDriveBehaviour);
    }
}
