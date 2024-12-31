package arachne4.lib.math;

import java.util.Map;
import java.util.function.BiFunction;
import java.util.function.DoubleFunction;
import java.util.function.ToDoubleFunction;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Interpolators {
    private static final Interpolator<Double> baseForDouble = Interpolator.forDouble();
    private static final InverseInterpolator<Double> baseForInverseDouble = InverseInterpolator.forDouble();

    public static final BidirectionalInterpolator<Double> forDouble = new BidirectionalInterpolator<>(baseForDouble, baseForInverseDouble);
    public static final DoubleBasedInterpolator<Rotation2d> forRotation2d = new DoubleBasedInterpolator<>(Rotation2d::getRadians, Rotation2d::fromRadians);

    public static final <U extends Unit<U>> DoubleBasedInterpolator<Measure<U>> forMeasure() {
        return new DoubleBasedInterpolator<Measure<U>>(Measure::baseUnitMagnitude, (magnitude, exampleMeasure) -> exampleMeasure.unit().ofBaseUnits(magnitude));
    }

    public static final <InputT, OutputT> InterpolatingTreeMap<InputT, OutputT> interpolateOver(
            InverseInterpolator<InputT> inverseInterpolator, Interpolator<OutputT> interpolator,
            Iterable<Map.Entry<InputT, OutputT>> interpolationPoints) {

        InterpolatingTreeMap<InputT, OutputT> map = new InterpolatingTreeMap<>(inverseInterpolator, interpolator);
        for (var entry : interpolationPoints) map.put(entry.getKey(), entry.getValue());

        return map;
    }

    public static final <InputT, OutputT> InterpolatingTreeMap<InputT, OutputT> interpolateOver(
            InverseInterpolator<InputT> inverseInterpolator, Interpolator<OutputT> interpolator,
            Map<InputT, OutputT> interpolationMap) {

        return interpolateOver(inverseInterpolator, interpolator, interpolationMap.entrySet());
    }

    public static final <InputT, OutputT> InterpolatingTreeMap<InputT, OutputT> interpolateOver(
            InverseInterpolator<InputT> inverseInterpolator, Interpolator<OutputT> interpolator,
            InputT startInput, OutputT startOutput,
            InputT endInput, OutputT endOutput) {

        return interpolateOver(inverseInterpolator, interpolator, Map.of(
            startInput, startOutput,
            endInput, endOutput
        ));
    }

    public static class BidirectionalInterpolator<T> implements Interpolator<T>, InverseInterpolator<T> {
        private final Interpolator<T> interpolator;
        private final InverseInterpolator<T> inverseInterpolator;

        public BidirectionalInterpolator(Interpolator<T> interpolator, InverseInterpolator<T> inverseInterpolator) {
            this.interpolator = interpolator;
            this.inverseInterpolator = inverseInterpolator;
        }

        @Override
        public T interpolate(T startValue, T endValue, double t) {
            return interpolator.interpolate(startValue, endValue, t);
        }

        @Override
        public double inverseInterpolate(T startValue, T endValue, T q) {
            return inverseInterpolator.inverseInterpolate(startValue, endValue, q);
        }
    }

    public static class DoubleBasedInterpolator<T> extends BidirectionalInterpolator<T> {
        public DoubleBasedInterpolator(ToDoubleFunction<T> toDouble, DoubleFunction<T> fromDouble) {
            super(
                (startValue, endValue, t) -> fromDouble.apply(baseForDouble.interpolate(toDouble.applyAsDouble(startValue), toDouble.applyAsDouble(endValue), t)),
                (startValue, endValue, q) -> baseForInverseDouble.inverseInterpolate(toDouble.applyAsDouble(startValue), toDouble.applyAsDouble(endValue), toDouble.applyAsDouble(q))
            );
        }

        public DoubleBasedInterpolator(ToDoubleFunction<T> toDouble, BiFunction<Double, T, T> fromDouble) {
            super(
                (startValue, endValue, t) -> fromDouble.apply(baseForDouble.interpolate(toDouble.applyAsDouble(startValue), toDouble.applyAsDouble(endValue), t), startValue),
                (startValue, endValue, q) -> baseForInverseDouble.inverseInterpolate(toDouble.applyAsDouble(startValue), toDouble.applyAsDouble(endValue), toDouble.applyAsDouble(q))
            );
        }
    }
}
