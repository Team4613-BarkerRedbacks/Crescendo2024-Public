package redbacks.robot.scorpion.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

public class ScorpionCameraConstants {
    static final String CAMERA_INPUTS_LOGGING_KEY = "Camera";

    static final Pose3d CENTER_CAMERA_OFFSET_FROM_ROBOT = new Pose3d(
        new Translation3d(Units.Meters.of(0.235), Units.Meters.zero(), Units.Meters.of(0.595)),
        new Rotation3d(0, Math.toRadians(30), Math.toRadians(180))
    );

    static final Pose3d LEFT_CAMERA_OFFSET_FROM_ROBOT = new Pose3d(
        new Translation3d(Units.Meters.of(0.195), Units.Meters.of(-0.12), Units.Meters.of(0.59)),
        new Rotation3d(0, Math.toRadians(30), Math.toRadians(135))
    );
}
