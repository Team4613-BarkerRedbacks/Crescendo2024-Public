package redbacks.robot.subsystems.drivetrain;

import java.util.Map;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.ToDoubleFunction;
import java.util.stream.Collectors;

import arachne4.lib.math.Interpolators;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;

public class HeadingControlUtils {
    public static Function<Translation2d, Rotation2d> aimAtPoint(Translation2d targetPoint) {
        return currentPosition -> targetPoint
            .minus(currentPosition)
            .getAngle();
    }

    public static Function<Translation2d, Rotation2d> aimAtPoint(Translation2d targetPoint, Rotation2d offsetToPoint) {
        return currentPosition -> targetPoint
            .minus(currentPosition)
            .getAngle()
            .plus(offsetToPoint);
    }

    public static Function<Translation2d, Rotation2d> aimAtPoint(Translation2d targetPoint, Supplier<Rotation2d> offsetToPointSupplier) {
        return currentPosition -> targetPoint
            .minus(currentPosition)
            .getAngle()
            .plus(offsetToPointSupplier.get());
    }

    public static Function<Translation2d, Rotation2d> aimAtPoint(Translation2d targetPoint, boolean frontToPoint) {
        return frontToPoint ? aimAtPoint(targetPoint)
            : currentPosition -> currentPosition
                .minus(targetPoint) // Direction of vector is flipped to face back of robot at point
                .getAngle();
    }

    public static Function<Translation2d, Rotation2d> aimAtPoint(Translation2d targetPoint, boolean frontToPoint, Rotation2d offsetToPoint) {
        return frontToPoint ? aimAtPoint(targetPoint, offsetToPoint)
            : currentPosition -> currentPosition
                .minus(targetPoint) // Direction of vector is flipped to face back of robot at point
                .getAngle()
                .plus(offsetToPoint);
    }

    public static Function<Translation2d, Rotation2d> aimAtPoint(Translation2d targetPoint, boolean frontToPoint, Supplier<Rotation2d> offsetToPointSupplier) {
        return frontToPoint ? aimAtPoint(targetPoint, offsetToPointSupplier)
            : currentPosition -> currentPosition
                .minus(targetPoint) // Direction of vector is flipped to face back of robot at point
                .getAngle()
                .plus(offsetToPointSupplier.get());
    }

    public static Function<Translation2d, Rotation2d> interpolateHeadingOnX(Map<Translation2d, Rotation2d> interpolationMap) {
        return interpolateHeadingOn(Translation2d::getX, interpolationMap);
    }

    public static Function<Translation2d, Rotation2d> interpolateHeadingOnY(Map<Translation2d, Rotation2d> interpolationMap) {
        return interpolateHeadingOn(Translation2d::getY, interpolationMap);
    }

    public static Function<Translation2d, Rotation2d> interpolateHeadingOn(
            ToDoubleFunction<Translation2d> positionToInterpolationInput,
            Map<Translation2d, Rotation2d> interpolationMap) {

        return interpolateHeadingWithDoubleMap(
            positionToInterpolationInput,
            interpolationMap.entrySet().stream().collect(Collectors.toMap(
                entry -> positionToInterpolationInput.applyAsDouble(entry.getKey()),
                Map.Entry::getValue)));
    }

    public static Function<Translation2d, Rotation2d> interpolateHeadingWithDoubleMap(
            ToDoubleFunction<Translation2d> positionToInterpolationInput,
            Map<Double, Rotation2d> interpolationMap) {

        // Generate the map once to reuse in all calls to the function
        InterpolatingTreeMap<Double, Rotation2d> treeMap = Interpolators.interpolateOver(
            Interpolators.forDouble,
            Interpolators.forRotation2d,
            interpolationMap);

        return currentPosition -> treeMap.get(positionToInterpolationInput.applyAsDouble(currentPosition));
    }
}
