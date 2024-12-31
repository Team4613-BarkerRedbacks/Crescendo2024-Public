package arachne4.lib.math;

import static org.junit.jupiter.api.Assertions.*;

import java.util.stream.Stream;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.units.Units;

public class TestArachneMath {
    @ParameterizedTest
    @MethodSource("provideAnglesFor_getClosestEquivalentAngleIsAlwaysWithin180Degrees")
    @Disabled("In favour of the space search test")
    void getClosestEquivalentAngleIsAlwaysWithin180Degrees(double value, double reference, double expected) {
        double result = ArachneMath.getClosestEquivalentAngle(value, reference, Units.Degrees);

        assertEquals(expected, result, 1e-6);
        assertTrue(Math.abs(reference - result) <= 180);
    }

    @Test
    void getClosestEquivalentAngleIsAlwaysWithin180DegreesForSpaceSearch() {
        int magnitudeToSearch = 360 * 3;
        double increment = 10 + Math.E;

        for (double value = -magnitudeToSearch; value <= magnitudeToSearch; value += increment) {
            for (double reference = -magnitudeToSearch; reference <= magnitudeToSearch; reference += increment) {
                double result = ArachneMath.getClosestEquivalentAngle(value, reference, Units.Degrees);
                assertTrue(Math.abs(reference - result) <= 180, String.format("|ref - result| > 180 for value = %f, reference = %f, result = %f", value, reference, result));
            }
        }
    }

    private static Stream<Arguments> provideAnglesFor_getClosestEquivalentAngleIsAlwaysWithin180Degrees() {
        return Stream.of(
            Arguments.of(1, 0, 1),
            Arguments.of(-1, 0, -1),
            Arguments.of(181, 0, -179),
            Arguments.of(Math.E, 360, 360 + Math.E),
            Arguments.of(181, -360 * 20, -360 * 20 - 179),
            Arguments.of(0, 1, 0),
            Arguments.of(0, -1, 0),
            Arguments.of(0, 181, 360)
        );
    }
}
