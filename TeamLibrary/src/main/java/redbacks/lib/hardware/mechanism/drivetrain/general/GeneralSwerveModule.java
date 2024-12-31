package redbacks.lib.hardware.mechanism.drivetrain.general;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import redbacks.lib.hardware.motor.SmartRotationalMotor;
import redbacks.lib.hardware.motor.SmartWheeledMotor;
import redbacks.lib.hardware.sensor.SmartAngleSensor;

public class GeneralSwerveModule implements Sendable {
    private final SmartWheeledMotor driveMotor;
    private final SmartRotationalMotor steerMotor;
    private final SmartAngleSensor encoder;
    private final Translation2d offsetFromRobotCentre;

    public GeneralSwerveModule(SmartWheeledMotor driveMotor, SmartRotationalMotor steerMotor, SmartAngleSensor encoder, Translation2d offsetFromRobotCentre) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.encoder = encoder;
        this.offsetFromRobotCentre = offsetFromRobotCentre;

        updateMotorAngleFromSensor();
    }

    public void updateMotorAngleFromSensor() {
        steerMotor.setSensorAngle(encoder.getAngle());
    }

    public Translation2d getOffsetFromRobotCentre() {
        return offsetFromRobotCentre;
    }

    public void drive(SwerveModuleState targetState) {
        if(targetState.speedMetersPerSecond != 0) {
            SwerveModuleState optimisedState = optimize(targetState);

            steerMotor.setTargetAngle(optimisedState.angle);
            driveMotor.setTargetVelocity(Units.MetersPerSecond.of(optimisedState.speedMetersPerSecond));
        }
        else {
            driveMotor.stop();
        }
    }

    private SwerveModuleState optimize(SwerveModuleState targetState) {
        double currentAngleDegrees = steerMotor.getAngle().getDegrees();
        double diff = targetState.angle.getDegrees() - currentAngleDegrees;

        double mod = MathUtil.inputModulus(diff, 0, 360);
        double speed = targetState.speedMetersPerSecond * Math.cos(Math.toRadians(mod));

        if(mod <= 90) return new SwerveModuleState(speed, Rotation2d.fromDegrees(currentAngleDegrees + mod));
        else if(mod <= 270) return new SwerveModuleState(speed, Rotation2d.fromDegrees(currentAngleDegrees + mod - 180));
        else return new SwerveModuleState(speed, Rotation2d.fromDegrees(currentAngleDegrees + mod - 360));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(driveMotor.getPosition().in(Units.Meters), steerMotor.getAngle());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(driveMotor.getVelocity().in(Units.MetersPerSecond), steerMotor.getAngle());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Module");
        builder.addDoubleProperty("Velocity (m|s)", () -> driveMotor.getVelocity().in(Units.MetersPerSecond), null);
        builder.addDoubleProperty("Motor angle (deg)", () -> steerMotor.getAngle().getDegrees(), null);
        builder.addDoubleProperty("Sensor angle (deg)", () -> encoder.getAngle().getDegrees(), null);
    }
}
