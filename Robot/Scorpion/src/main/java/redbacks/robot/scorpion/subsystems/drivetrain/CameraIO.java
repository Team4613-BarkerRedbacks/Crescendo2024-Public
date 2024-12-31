package redbacks.robot.scorpion.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.geometry.Pose2d;
import redbacks.lib.hardware.vision.limelight.MegatagData;
import redbacks.lib.io.InputsProvider;

public abstract class CameraIO extends InputsProvider<CameraInputsAutoLogged> {
    @AutoLog
    public static class CameraInputs {
        MegatagData centerMegatag = new MegatagData(new Pose2d(), 0, 0, 0, new int[0]);
        MegatagData centerMegatag2 = new MegatagData(new Pose2d(), 0, 0, 0, new int[0]);
        MegatagData leftMegatag = new MegatagData(new Pose2d(), 0, 0, 0, new int[0]);
        MegatagData leftMegatag2 = new MegatagData(new Pose2d(), 0, 0, 0, new int[0]);
    }

    public CameraIO(Scheduler scheduler) {
        super(scheduler, ScorpionCameraConstants.CAMERA_INPUTS_LOGGING_KEY, new CameraInputsAutoLogged());
    }

    public static class Empty extends CameraIO {
        public Empty(Scheduler scheduler) {
            super(scheduler);
        }

        @Override public void updateInputs() {}
    }
}
