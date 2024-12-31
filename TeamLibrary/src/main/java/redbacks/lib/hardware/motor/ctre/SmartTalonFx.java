package redbacks.lib.hardware.motor.ctre;

import java.util.Optional;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import redbacks.lib.hardware.motor.SmartRotationalMotor;
import redbacks.lib.hardware.motor.SmartWheeledMotor;
import redbacks.lib.math.characteristics.GearRatio;
import redbacks.lib.math.characteristics.PidfConfig;

public class SmartTalonFx {

    private final TalonFxMotorGroup motors;
    private final boolean useFieldOrientedControl;

    public SmartTalonFx(TalonFxMotorGroup motors, boolean useFieldOrientedControl) {
        this.motors = motors;
        this.useFieldOrientedControl = useFieldOrientedControl;
    }

    public void setPercentageOutput(double power) {
        motors.setPercentageOutput(new DutyCycleOut(power, useFieldOrientedControl, false, false, false));
    }

    public SmartRotationalMotor createRotationalMotor(GearRatio gearRatio, PidfConfig pidConfig, boolean useMotionMagicForPositionControl) {
        motors.configPid(pidConfig);

        return new SmartRotationalMotor() {
            Rotation2d targetAngle;

            @Override
            public void setPercentageOutput(double power) {
                SmartTalonFx.this.setPercentageOutput(power);
                targetAngle = null;
            }

            @Override
            public void setToCoast() {
                motors.setNeutralMode(NeutralModeValue.Coast);
            }

            @Override
            public void setToBrake() {
                motors.setNeutralMode(NeutralModeValue.Brake);
            }

            @Override
            public Rotation2d getAngle() {
                return fromRevolutions(motors.getSelectedSensorPosition(), gearRatio);
            }

            @Override
            public Optional<Rotation2d> getTargetAngle() {
                return Optional.ofNullable(targetAngle);
            }

            @Override
            public void setSensorAngle(Rotation2d angle) {
                motors.setSelectedSensorPosition(toRevolutions(angle, gearRatio));
            }

            @Override
            public void setTargetAngle(Rotation2d angle) {
                setTargetAngle(angle, 0);
            }

            @Override
            public void setTargetAngle(Rotation2d angle, double arbitraryFeedForwardPercent) {
                double target = toRevolutions(angle, gearRatio);

                if(useMotionMagicForPositionControl) {
                    motors.setTargetRotation(new MotionMagicDutyCycle(
                        target,
                        useFieldOrientedControl,
                        arbitraryFeedForwardPercent,
                        0,
                        false,
                        false,
                        false
                    ));
                }
                else {
                    motors.setTargetRotation(new PositionDutyCycle(
                        target,
                        0,
                        useFieldOrientedControl,
                        arbitraryFeedForwardPercent,
                        0,
                        false,
                        false,
                        false
                    ));
                }

                targetAngle = angle;
            }

            @Override
            public void stop() {
                motors.setPercentageOutput(new DutyCycleOut(0));
                targetAngle = null;
            }
        };
    }

    public SmartWheeledMotor createEmpiricallyDistancedMotor(double revolutionsPerMetre, PidfConfig pidConfig, boolean useMotionMagicForPositionControl) {
        motors.configPid(pidConfig);

        return new SmartWheeledMotor() {
            Measure<Distance> targetPosition;
            Measure<Velocity<Distance>> targetVelocity;

            @Override
            public void setPercentageOutput(double power) {
                SmartTalonFx.this.setPercentageOutput(power);
                targetPosition = null;
                targetVelocity = null;
            }

            @Override
            public void setToCoast() {
                motors.setNeutralMode(NeutralModeValue.Coast);
            }

            @Override
            public void setToBrake() {
                motors.setNeutralMode(NeutralModeValue.Brake);
            }

            @Override
            public Measure<Distance> getPosition() {
                return Units.Meters.of(motors.getSelectedSensorPosition() / revolutionsPerMetre);
            }

            @Override
            public void setCurrentPosition(Measure<Distance> position) {
                motors.setSelectedSensorPosition(position.in(Units.Meters) * revolutionsPerMetre);
            }

            @Override
            public void setTargetPosition(Measure<Distance> position) {
                setTargetPosition(position, 0);
            }

            @Override
            public void setTargetPosition(Measure<Distance> position, double arbitraryFeedForwardPercent) {
                double target = position.in(Units.Meters) * revolutionsPerMetre;

                if(useMotionMagicForPositionControl) {
                    motors.setTargetRotation(new MotionMagicDutyCycle(
                        target,
                        useFieldOrientedControl,
                        arbitraryFeedForwardPercent,
                        0,
                        false,
                        false,
                        false
                    ));
                }
                else {
                    motors.setTargetRotation(new PositionDutyCycle(
                        target,
                        0,
                        useFieldOrientedControl,
                        arbitraryFeedForwardPercent,
                        0,
                        false,
                        false,
                        false
                    ));
                }

                targetPosition = position;
                targetVelocity = null;
            }

            @Override
            public Measure<Velocity<Distance>> getVelocity() {
                return Units.MetersPerSecond.of(motors.getSelectedSensorVelocity() / revolutionsPerMetre);
            }

            @Override
            public void setTargetVelocity(Measure<Velocity<Distance>> velocity) {
                motors.setTargetVelocityRps(new VelocityDutyCycle(
                    velocity.in(Units.MetersPerSecond) * revolutionsPerMetre,
                    0,
                    useFieldOrientedControl,
                    0,
                    0,
                    false,
                    false,
                    false
                ));

                targetPosition = null;
                targetVelocity = velocity;
            }

            @Override
            public void stop() {
                motors.setPercentageOutput(new DutyCycleOut(0));
                targetPosition = null;
                targetVelocity = null;
            }

            @Override
            public double getRawVelocity() {
                return motors.getSelectedSensorVelocity();
            }

            @Override
            public Optional<Measure<Distance>> getTargetPosition() {
                return Optional.ofNullable(targetPosition);
            }

            @Override
            public Optional<Measure<Velocity<Distance>>> getTargetVelocity() {
                return Optional.ofNullable(targetVelocity);
            }
        };
    }

    public SmartWheeledMotor createWheeledMotor(Measure<Distance> wheelDiameter, GearRatio gearRatio, PidfConfig pidConfig, boolean useMotionMagicForPositionControl) {
        var wheelCircumference = wheelDiameter.times(Math.PI);
        var revolutionsPerMetre = gearRatio.getReduction() / wheelCircumference.in(Units.Meters);

        return createEmpiricallyDistancedMotor(revolutionsPerMetre, pidConfig, useMotionMagicForPositionControl);
    }

    public static double getVelocityForConfig(Measure<Velocity<Angle>> velocity, GearRatio motorGearRatio) {
        return toRevolutions(Rotation2d.fromRotations(velocity.in(Units.RotationsPerSecond)), motorGearRatio);
    }

    public static double getAccelerationForConfig(Measure<Velocity<Velocity<Angle>>> acceleration, GearRatio motorGearRatio) {
        return toRevolutions(Rotation2d.fromRotations(acceleration.in(Units.RotationsPerSecond.per(Units.Seconds))), motorGearRatio);
    }

    private static double toRevolutions(Rotation2d angle, GearRatio motorGearRatio) {
        return motorGearRatio.getInputCountFromOutput(angle.getRotations());
    }

    private static Rotation2d fromRevolutions(double revolutions, GearRatio motorGearRatio) {
        return Rotation2d.fromRotations(motorGearRatio.getOutputCountFromInput(revolutions));
    }

}
