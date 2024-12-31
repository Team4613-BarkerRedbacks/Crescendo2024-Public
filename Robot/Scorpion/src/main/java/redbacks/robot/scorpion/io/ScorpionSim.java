package redbacks.robot.scorpion.io;

import arachne4.lib.Constants;
import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import redbacks.lib.subsystems.positionbased.AngleBasedIO;
import redbacks.lib.subsystems.positionbased.DistanceBasedIO;
import redbacks.robot.Controllers;
import redbacks.robot.scorpion.subsystems.climber.ScorpionClimberSim;
import redbacks.robot.scorpion.subsystems.drivetrain.CameraIO;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerIO;
import redbacks.robot.scorpion.subsystems.noteHandling.blower.BlowerSim;
import redbacks.robot.sim.FieldSim;
import redbacks.robot.sim.NoteSim;
import redbacks.robot.subsystems.drivetrain.DrivetrainConstants;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;
import redbacks.robot.subsystems.drivetrain.DrivetrainSim;
import redbacks.robot.subsystems.noteHandling.NoteHandlingIO;
import redbacks.robot.subsystems.noteHandling.NoteHandlingSim;
import redbacks.robot.subsystems.noteHandling.pivot.PivotSim;

public class ScorpionSim extends SchedulerProviderBase implements ScorpionIOProvider {
    private static final DrivetrainConstants DRIVETRAIN_CONSTANTS = Constants.get(DrivetrainConstants.class);

    private final Controllers controllers;

    private final DrivetrainSim.WithFullSimulation drivetrain;
    private final CameraIO camera;
    private final NoteHandlingSim noteHandling;
    private final PivotSim pivot;
    private final BlowerSim blower;
    private final ScorpionClimberSim climber;

    private final FieldSim field;

    public ScorpionSim(Scheduler scheduler) {
        super(scheduler);

        this.controllers = Controllers.getInstance();

        this.drivetrain = new DrivetrainSim.WithFullSimulation(scheduler);
        this.camera = new CameraIO.Empty(scheduler);
        this.pivot = new PivotSim(scheduler);
        this.blower = new BlowerSim(scheduler);
        this.noteHandling = new NoteHandlingSim(scheduler, pivot);
        this.climber = new ScorpionClimberSim(scheduler);

        this.field = new FieldSim(scheduler);
        this.field.resetWithStandardStagedNotes(DriverStation.getAlliance().orElse(Alliance.Red));
    }

    @Override
    public DrivetrainIO getDrivetrainIO() {
        return drivetrain;
    }

    @Override
    public CameraIO getCameraIO() {
        return camera;
    }

    @Override
    public NoteHandlingIO getNoteHandlingIO() {
        return noteHandling;
    }

    @Override
    public AngleBasedIO getPivotIO() {
        return pivot;
    }

    @Override
    public BlowerIO getBlowerIO() {
        return blower;
    }

    @Override
    public DistanceBasedIO getClimberIO() {
        return climber;
    }

    { registerHandler(Scheduler.SIMULATION_EXECUTE, this::update); }
    private void update() {
        if (controllers.driver.getBackButtonPressed()) {
            var robotPose = drivetrain.getCurrentPose();
            field.addNote(new NoteSim(robotPose.getTranslation().plus(new Translation2d(Units.Meters.of(0.5), Units.Meters.zero()).rotateBy(robotPose.getRotation()))));
        }

        updateHeldNote();

        for (var note : field.getNotes()) {
            if (shouldNoteBePickedUp(note)) {
                intakeNote(note);
            }
        }
    }

    { registerHandler(Scheduler.GAME_STATE_CHANGE, to(GameState.AUTO), this::reset); }
    public void reset() {
        field.resetWithStandardStagedNotes(DriverStation.getAlliance().orElse(Alliance.Red));

        var preload = new NoteSim(new Pose3d());
        noteHandling.preload(preload);
        field.addNote(preload);
    }

    private boolean shouldNoteBePickedUp(NoteSim note) {
        var currentRobotPos = drivetrain.getCurrentPose();
        var currentIntakeLocation = currentRobotPos.getTranslation()
            .plus(new Translation2d(DRIVETRAIN_CONSTANTS.getRobotOffsetXFromEdge(), Units.Meters.zero()).rotateBy(currentRobotPos.getRotation()));

        return note.canBePickedUp()
            && note.getCurrentPose().toPose2d().getTranslation().getDistance(currentIntakeLocation) < 0.3
            && noteHandling.isIntaking()
            && !noteHandling.hasNote();
    }

    private void intakeNote(NoteSim note) {
        noteHandling.intakeNote(note, drivetrain.getCurrentPose());
    }

    private void updateHeldNote() {
        noteHandling.updateHeldNotePos(drivetrain.getCurrentPose(), drivetrain.getFieldRelativeVelocity());
    }
}
