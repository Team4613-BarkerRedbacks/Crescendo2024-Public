package redbacks.robot.subsystems.noteHandling.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import redbacks.robot.subsystems.noteHandling.pivot.PivotMappings.ManualInputConstants;
import redbacks.robot.subsystems.noteHandling.pivot.PivotHardware.HardwareConstants;
import redbacks.robot.subsystems.noteHandling.pivot.PivotSim.SimConstants;

public interface PivotConstants {
    String INPUTS_LOGGING_KEY = "Pivot";

    HardwareConstants hardwareConstants();
    SimConstants simConstants();
    ManualInputConstants manualInputConstants();

    Rotation2d minimumAngle();
    Rotation2d maximumAngle();
    Rotation2d hardStopAngle();
    Rotation2d manualEndpointRange();
    InterpolatingTreeMap<Double, Rotation2d> feedToCenterAnglesFromXVelocity();
    InterpolatingTreeMap<Double, Rotation2d> feedToAmpAnglesFromXVelocity();
    Rotation2d subwooferAngle();
    Rotation2d ampAngle();
    Rotation2d lowAngle();
}
