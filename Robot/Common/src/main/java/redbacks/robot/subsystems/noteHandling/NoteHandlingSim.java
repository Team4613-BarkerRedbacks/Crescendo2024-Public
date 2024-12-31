package redbacks.robot.subsystems.noteHandling;

import java.util.Optional;

import arachne4.lib.Constants;
import arachne4.lib.scheduler.Scheduler;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import redbacks.robot.CommonConstants;
import redbacks.robot.sim.NoteSim;
import redbacks.robot.subsystems.noteHandling.NoteHandlingConstants.ShooterConstants;
import redbacks.robot.subsystems.noteHandling.pivot.PivotSim;

public class NoteHandlingSim extends NoteHandlingIO {
    private static final ShooterConstants SHOOTER_CONSTANTS = Constants.get(ShooterConstants.class);

    private final PivotSim pivotSim;

    private final FlywheelSim leftShooterSim = new FlywheelSim(
        DCMotor.getFalcon500(1),
        SHOOTER_CONSTANTS.gearRatio().getReduction(),
        0.005);

    private final FlywheelSim rightShooterSim = new FlywheelSim(
        DCMotor.getFalcon500(1),
        SHOOTER_CONSTANTS.gearRatio().getReduction(),
        0.005);

    private final PIDController leftShooterPidController = new PIDController(10, 0, 0, CommonConstants.LOOP_PERIOD.in(Units.Seconds));
    private final double leftFeedForwardCoefficient = 12d / 40d;
    private boolean leftShooterPidControllerEnabled = false;

    private final PIDController rightShooterPidController = new PIDController(10, 0, 0, CommonConstants.LOOP_PERIOD.in(Units.Seconds));
    private final double rightFeedForwardCoefficient = 12d / 40d;
    private boolean rightShooterPidControllerEnabled = false;

    private double intakePower = 0;

    private Optional<NoteSim> heldNote = Optional.empty();
    private HeldNoteState heldNoteState = null;
    private double heldNoteEnteredStateTime;
    private Transform3d noteInitialRelativePos;

    public NoteHandlingSim(Scheduler scheduler, PivotSim pivotSim) {
        super(scheduler);

        this.pivotSim = pivotSim;
    }

    @Override
    public void updateInputs() {
        updateLeftShooter();
        updateRightShooter();

        inputs.leftShooterVelocity = Units.MetersPerSecond.of(radiansToMeters(leftShooterSim.getAngularVelocityRadPerSec()));
        inputs.rightShooterVelocity = Units.MetersPerSecond.of(radiansToMeters(rightShooterSim.getAngularVelocityRadPerSec()));

        inputs.isNoteInStow = heldNote.isPresent() && heldNoteState == HeldNoteState.STOWED;
        inputs.isNoteInIntake = heldNote.isPresent() && heldNoteState == HeldNoteState.INTAKING;
    }

    private void updateLeftShooter() {
        if (leftShooterPidControllerEnabled) {
            var volts = leftShooterPidController.calculate(radiansToMeters(leftShooterSim.getAngularVelocityRadPerSec()))
                + leftFeedForwardCoefficient * leftShooterPidController.getSetpoint();

            leftShooterSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
        }

        leftShooterSim.update(CommonConstants.LOOP_PERIOD.in(edu.wpi.first.units.Units.Seconds));
    }

    private void updateRightShooter() {
        if (rightShooterPidControllerEnabled) {
            var volts = rightShooterPidController.calculate(radiansToMeters(rightShooterSim.getAngularVelocityRadPerSec()))
                + rightFeedForwardCoefficient * rightShooterPidController.getSetpoint();

            rightShooterSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
        }

        rightShooterSim.update(CommonConstants.LOOP_PERIOD.in(edu.wpi.first.units.Units.Seconds));
    }

    @Override
    public void setLeftShooterTargetVelocity(Measure<Velocity<Distance>> speed) {
        leftShooterPidControllerEnabled = true;
        leftShooterPidController.setSetpoint(speed.in(Units.MetersPerSecond));
    }

    @Override
    public void setRightShooterTargetVelocity(Measure<Velocity<Distance>> speed) {
        rightShooterPidControllerEnabled = true;
        rightShooterPidController.setSetpoint(speed.in(Units.MetersPerSecond));
    }

    @Override
    public void setIntakePercentageOutput(double power) {
        intakePower = power;
    }

    public void updateHeldNotePos(Pose2d drivetrainPos, ChassisSpeeds fieldRelativeSpeed) {
        if (!heldNote.isPresent()) return;

        var note = heldNote.get();
        switch (heldNoteState) {
            case INTAKING:
                double percentageIntaked = Math.min(1, (Timer.getFPGATimestamp() - heldNoteEnteredStateTime) / 0.2);
                note.setPose(getStowedPos(drivetrainPos).plus(noteInitialRelativePos.times(1 - percentageIntaked)));

                if (percentageIntaked == 1) {
                    heldNoteState = HeldNoteState.STOWED;
                    heldNoteEnteredStateTime = Timer.getFPGATimestamp();
                }

                break;

            case STOWED:
                note.setPose(getStowedPos(drivetrainPos));
                if (isIntaking()) {
                    heldNoteState = HeldNoteState.SHOOTING;
                    heldNoteEnteredStateTime = Timer.getFPGATimestamp();
                }
                break;

            case SHOOTING:
                note.setPose(getStowedPos(drivetrainPos));

                if (Timer.getFPGATimestamp() - heldNoteEnteredStateTime > 0.05) {
                    note.launch(
                        drivetrainPos.getRotation().rotateBy(Rotation2d.fromDegrees(180)),
                        fieldRelativeSpeed,
                        radiansToMeters(leftShooterSim.getAngularVelocityRadPerSec() + rightShooterSim.getAngularVelocityRadPerSec()) / 2,
                        pivotSim.getAngle());

                    heldNote = Optional.empty();
                }

                break;
        }
    }

    private Pose3d getStowedPos(Pose2d drivetrainPos) {
        return new Pose3d(drivetrainPos).plus(
            new Transform3d(0, 0, 0.2,
            new Rotation3d(0, pivotSim.getAngle().getRadians(), 0)));
    }

    public void intakeNote(NoteSim note, Pose2d drivetrainPos) {
        heldNote = Optional.of(note);
        heldNoteState = HeldNoteState.INTAKING;
        heldNoteEnteredStateTime = Timer.getFPGATimestamp();
        noteInitialRelativePos = note.getCurrentPose().minus(getStowedPos(drivetrainPos));
    }

    public void preload(NoteSim note) {
        heldNote = Optional.of(note);
        heldNoteState = HeldNoteState.STOWED;
        heldNoteEnteredStateTime = Timer.getFPGATimestamp();
    }

    public boolean isIntaking() {
        return intakePower < 0;
    }

    public boolean hasNote() {
        return heldNote.isPresent();
    }

    private double radiansToMeters(double radians) {
        return radians * SHOOTER_CONSTANTS.wheelDiameter().in(Units.Meters) / 2;
    }

    private static enum HeldNoteState {
        INTAKING,
        STOWED,
        SHOOTING;
    }
}
