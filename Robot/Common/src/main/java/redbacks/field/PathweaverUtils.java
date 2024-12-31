package redbacks.field;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.LinkedList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import arachne4.lib.logging.ArachneLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class PathweaverUtils {
    public static final ControlVectorList readControlVectorsForBlue(String pathweaverFilename) {
        return readControlVectors(pathweaverFilename, false);
    }

    public static final ControlVectorList readControlVectorsForRed(String pathweaverFilename) {
        return readControlVectors(pathweaverFilename, true);
    }

    private static final ControlVectorList readControlVectors(String pathweaverFilename, boolean flipY) {
        try(BufferedReader br = new BufferedReader(new FileReader(new File(
            Filesystem.getDeployDirectory(),
            pathweaverFilename)))) {

            var controlVectors = new ControlVectorList();

            // Ignore title row
            br.readLine();

            for(String line = br.readLine(); line != null; line = br.readLine()) {
                var values = line.split(",");

                var x = Double.parseDouble(values[0]);
                var y = FieldLocations.FIELD_WIDTH.in(Units.Meters) + Double.parseDouble(values[1]);
                var tangentX = Double.parseDouble(values[2]);
                var tangentY = Double.parseDouble(values[3]);

                if(flipY) {
                    y *= -1;
                    tangentY *= -1;
                }

                controlVectors.add(new ControlVector(
                    new double[] { x, tangentX, 0 },
                    new double[] { y, tangentY, 0 }
                ));
            }

            return controlVectors;
        }
        catch(IOException ex) {
            ArachneLogger.getInstance().critical("Unable to open pathweaver file: " + pathweaverFilename);
            return null;
        }
    }

    public static final Trajectory readForBlue(String trajectoryFilename) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilename);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        }
        catch(IOException ex) {
            ArachneLogger.getInstance().critical("Unable to open trajectory: " + trajectoryFilename);
            return null;
        }
    }

    public static final Trajectory readForRed(String trajectoryFilename) {
        try {
            var trajectoryFile = new File(Filesystem.getDeployDirectory(), trajectoryFilename);
            var jsonMapper = new ObjectMapper();

            var points = new LinkedList<JsonNode>();
            jsonMapper.readTree(trajectoryFile).elements().forEachRemaining(points::add);

            var flattened = new double[points.size() * 7];

            for(var i = 0; i < flattened.length / 7; i++) {
                var point = points.removeFirst();

                var acceleration = point.get("acceleration").asDouble();
                var curvature = point.get("curvature").asDouble();

                var pose = point.get("pose");
                var rotation = pose.get("rotation").get("radians").asDouble();

                var translation = pose.get("translation");
                var x = translation.get("x").asDouble();
                var y = translation.get("y").asDouble();

                var time = point.get("time").asDouble();
                var velocity = point.get("velocity").asDouble();

                flattened[i * 7 + 0] = time;

                flattened[i * 7 + 1] = velocity;
                flattened[i * 7 + 2] = acceleration;

                flattened[i * 7 + 3] = x;
                flattened[i * 7 + 4] = -y;
                flattened[i * 7 + 5] = -rotation;

                flattened[i * 7 + 6] = -curvature;
            }

            return createTrajectoryFromElements(flattened);
        }
        catch(IOException ex) {
            ArachneLogger.getInstance().critical("Unable to open trajectory: " + trajectoryFilename);
            return null;
        }
    }

    /**
     * Based on {@link TrajectoryUtil.createTrajectoryFromElements}.
     */
    private static Trajectory createTrajectoryFromElements(double[] elements) {
        List<Trajectory.State> states = new LinkedList<>();

        for(int i = 0; i < elements.length; i += 7) {
            states.add(new Trajectory.State(
                elements[i],
                elements[i + 1],
                elements[i + 2],
                new Pose2d(elements[i + 3], elements[i + 4], Rotation2d.fromRadians(elements[i + 5])),
                elements[i + 6]
            ));
        }

        return new Trajectory(states);
    }
}
