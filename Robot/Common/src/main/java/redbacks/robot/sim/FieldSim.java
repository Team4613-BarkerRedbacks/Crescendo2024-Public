package redbacks.robot.sim;

import static redbacks.field.FieldLocations.*;

import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProviderBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldSim extends SchedulerProviderBase {
    private final Set<NoteSim> notes;
    private final Queue<NoteSim> notesToRemove = new LinkedList<>();

    public FieldSim(Scheduler scheduler) {
        super(scheduler);
        this.notes = new HashSet<>();
    }

    public void resetWithStandardStagedNotes(Alliance alliance) {
        notes.clear();

        Stream.of(
                WING_NOTE_1, WING_NOTE_2, WING_NOTE_3,
                CENTER_NOTE_1, CENTER_NOTE_2, CENTER_NOTE_3, CENTER_NOTE_4, CENTER_NOTE_5)
            .map(note -> note.in(alliance))
            .map(NoteSim::new)
            .forEach(notes::add);
    }

    public Collection<NoteSim> getNotes() {
        return notes;
    }

    public void addNote(NoteSim note) {
        notes.add(note);
    }

    public void removeNote(NoteSim note) {
        notesToRemove.add(note);
    }

    { registerHandler(Scheduler.SIMULATION_EXECUTE, this::update); }
    private void update() {
        for (var note : notes) {
            note.update();

            var notePos = note.getCurrentPose().toPose2d().getTranslation();
            if (notePos.getX() < 0) removeNote(note);
        }

        for (var note : notesToRemove) notes.remove(note);

        Logger.recordOutput("FieldSim/Notes/Poses", notes.stream()
            .map(NoteSim::getCurrentPose)
            .toArray(Pose3d[]::new));
    }
}
