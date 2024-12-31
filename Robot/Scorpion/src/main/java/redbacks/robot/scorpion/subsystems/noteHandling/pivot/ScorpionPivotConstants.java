package redbacks.robot.scorpion.subsystems.noteHandling.pivot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import arachne4.lib.Constants.RobotConstants;
import arachne4.lib.game.AllianceSpecific;
import arachne4.lib.math.Interpolators;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.hardware.motor.ctre.SmartTalonFx;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;
import redbacks.robot.scorpion.Scorpion;
import redbacks.robot.subsystems.noteHandling.pivot.PivotConstants;
import redbacks.robot.subsystems.noteHandling.pivot.PivotHardware.HardwareConstants;
import redbacks.robot.subsystems.noteHandling.pivot.PivotMappings.ManualInputConstants;
import redbacks.robot.subsystems.noteHandling.pivot.PivotSim.SimConstants;

@RobotConstants(Scorpion.class)
public class ScorpionPivotConstants implements PivotConstants {
    public static final ManualInputConstants MANUAL_INPUT_CONSTANTS = new ManualInputConstants(0.3, 0.2);

    static final Rotation2d
        HARD_STOP_ANGLE = Rotation2d.fromDegrees(21),
        MINIMUM_ANGLE = Rotation2d.fromDegrees(24),
        MAXIMUM_ANGLE = Rotation2d.fromDegrees(85),
        LOW_ANGLE = Rotation2d.fromDegrees(24),
        AMP_ANGLE = Rotation2d.fromDegrees(66),
        TRAP_ANGLE = Rotation2d.fromDegrees(64),
        MANUAL_ENDPOINT_RANGE = Rotation2d.fromDegrees(0.5);

    public static final Rotation2d SUBWOOFER_ANGLE = Rotation2d.fromDegrees(61.5);

    static final AllianceSpecific<Rotation2d> 
        ampFixedShootingPositionRobotHeading = AllianceSpecific.forRed(Rotation2d.fromDegrees(90)).forBlue(Rotation2d.fromDegrees(-90));

    static final GearRatio GEAR_RATIO = new GearRatio(225, 1);

    static final double TARGET_TOLERANCE = 0.5;

    static final PidfConfig PID_CONFIG = new PidfConfig(
        0.2,
        0,
        0,
        0.2 / 15
    );

    static final Measure<Velocity<Angle>> CRUISE_VELOCITY = Units.RotationsPerSecond.of(0.5);
    static final Measure<Velocity<Velocity<Angle>>> MAX_ACCELERATION = CRUISE_VELOCITY.per(Units.Second).divide(0.5);
    static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(SmartTalonFx.getVelocityForConfig(CRUISE_VELOCITY, GEAR_RATIO))
            .withMotionMagicAcceleration(SmartTalonFx.getAccelerationForConfig(MAX_ACCELERATION, GEAR_RATIO)))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));

    public static final HardwareConstants HARDWARE_CONSTANTS = new HardwareConstants(8, MOTOR_CONFIG, GEAR_RATIO, PID_CONFIG);
    public static final SimConstants SIM_CONSTANTS = new SimConstants(GEAR_RATIO, CRUISE_VELOCITY, MAX_ACCELERATION, HARD_STOP_ANGLE, MAXIMUM_ANGLE);

    private static final Interpolator<Double> doubleInterpolator = Interpolator.forDouble();
    static final AllianceSpecific<InterpolatingTreeMap<Double, Rotation2d>> DISTANCE_TO_ANGLE_MAP = AllianceSpecific.<InterpolatingTreeMap<Double, Rotation2d>>
        forRed(new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (startValue, endValue, t) -> Rotation2d.fromRadians(doubleInterpolator.interpolate(startValue.getRadians(), endValue.getRadians(), t)))
        {{
            put(1.34, Rotation2d.fromDegrees(64.2));
            put(1.82, Rotation2d.fromDegrees(55.7));
            put(2.11, Rotation2d.fromDegrees(49));
            put(3.4, Rotation2d.fromDegrees(34.3));
            put(4.42, Rotation2d.fromDegrees(28.9));
            put(4.7, Rotation2d.fromDegrees(27.8));
            put(5.38, Rotation2d.fromDegrees(26.1));
            put(5.73, Rotation2d.fromDegrees(25.9));

        }})
        .forBlue(new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), (startValue, endValue, t) -> Rotation2d.fromRadians(doubleInterpolator.interpolate(startValue.getRadians(), endValue.getRadians(), t)))
        {{
            put(1.34, Rotation2d.fromDegrees(64.2));
            put(1.82, Rotation2d.fromDegrees(55.7));
            put(2.11, Rotation2d.fromDegrees(49));
            put(3.4, Rotation2d.fromDegrees(34.3));
            put(4.42, Rotation2d.fromDegrees(28.9));
            put(4.7, Rotation2d.fromDegrees(27.8));
            put(5.38, Rotation2d.fromDegrees(26.1));
            put(5.73, Rotation2d.fromDegrees(25.9));
        }});


        // ----- TREES -----

    static final InterpolatingTreeMap<Double, Rotation2d> FEED_TO_CENTER_ANGLES_FROM_X_VELOCITY =
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forRotation2d)
        {{
            put(0.0, Rotation2d.fromDegrees(55));
            put(-2.0, Rotation2d.fromDegrees(65));
        }};

    static final InterpolatingTreeMap<Double, Rotation2d> FEED_TO_AMP_ANGLES_FROM_X_VELOCITY = 
        new InterpolatingTreeMap<>(Interpolators.forDouble, Interpolators.forRotation2d)
        {{
            put(0.0, Rotation2d.fromDegrees(45));
            put(-2.0, Rotation2d.fromDegrees(55));
        }};

    @Override
    public HardwareConstants hardwareConstants() {
        return HARDWARE_CONSTANTS;
    }

    @Override
    public SimConstants simConstants() {
        return SIM_CONSTANTS;
    }

    @Override
    public Rotation2d minimumAngle() {
        return MINIMUM_ANGLE;
    }

    @Override
    public Rotation2d maximumAngle() {
        return MAXIMUM_ANGLE;
    }

    @Override
    public Rotation2d hardStopAngle() {
        return HARD_STOP_ANGLE;
    }

    @Override
    public Rotation2d manualEndpointRange() {
        return MANUAL_ENDPOINT_RANGE;
    }

    @Override
    public InterpolatingTreeMap<Double, Rotation2d> feedToCenterAnglesFromXVelocity() {
        return FEED_TO_CENTER_ANGLES_FROM_X_VELOCITY;
    }

    @Override
    public InterpolatingTreeMap<Double, Rotation2d> feedToAmpAnglesFromXVelocity() {
        return FEED_TO_AMP_ANGLES_FROM_X_VELOCITY;
    }

    @Override
    public Rotation2d subwooferAngle() {
        return SUBWOOFER_ANGLE;
    }

    @Override
    public Rotation2d ampAngle() {
        return AMP_ANGLE;
    }

    @Override
    public Rotation2d lowAngle() {
        return LOW_ANGLE;
    }

    @Override
    public ManualInputConstants manualInputConstants() {
        return MANUAL_INPUT_CONSTANTS;
    }
}
