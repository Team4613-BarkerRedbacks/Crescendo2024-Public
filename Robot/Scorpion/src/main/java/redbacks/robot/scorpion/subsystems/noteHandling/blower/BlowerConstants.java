package redbacks.robot.scorpion.subsystems.noteHandling.blower;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.hardware.motor.ctre.SmartTalonFx;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;

public class BlowerConstants {
    // 0 is vertically down
    static final Rotation2d
        BLOW_ANGLE = Rotation2d.fromDegrees(45),
        STOW_ANGLE = Rotation2d.fromDegrees(5),
        CLIMB_ANGLE = Rotation2d.fromDegrees(90),
        ROTATION_MINUMUM = Rotation2d.fromDegrees(0),
        ROTATION_MAXIMUM = Rotation2d.fromDegrees(90),
        TARGET_TOLERANCE = Rotation2d.fromDegrees(5);

    static final double BLOW_POWER = -1;

    static final GearRatio ARM_GEAR_RATIO = new GearRatio(25 * 48, 18); 
    
    static final PidfConfig ARM_PID_CONFIG = new PidfConfig(
        0.02,
        0,
        0,
        0.4 / 38
    ); 

    static final Measure<Velocity<Angle>> ARM_CRUISE_VELOCITY = Units.RotationsPerSecond.of(1.2);
    static final Measure<Velocity<Velocity<Angle>>> ARM_MAX_ACCELERATION = ARM_CRUISE_VELOCITY.per(Units.Second).divide(0.3);

    static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(SmartTalonFx.getVelocityForConfig(ARM_CRUISE_VELOCITY, ARM_GEAR_RATIO))
            .withMotionMagicAcceleration(SmartTalonFx.getAccelerationForConfig(ARM_MAX_ACCELERATION, ARM_GEAR_RATIO)))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive));
}