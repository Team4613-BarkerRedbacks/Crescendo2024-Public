package arachne4.lib.game;

import java.util.function.Function;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceSpecific<ValueT> {
    private final ValueT valueforRed;
    private final ValueT valueforBlue;

    private AllianceSpecific(ValueT valueforRed, ValueT valueforBlue) {
        this.valueforRed = valueforRed;
        this.valueforBlue = valueforBlue;
    }

    public ValueT in(Alliance alliance) {
        return alliance == Alliance.Blue ? valueforBlue : valueforRed;
    }

    public static <ValueT> AllianceSpecificBuilder<ValueT> forRed(ValueT valueForRed) {
        return new AllianceSpecificBuilder<>(valueForRed);
    }

    public static <ValueT> AllianceSpecific<ValueT> forAlliance(Function<Alliance, ValueT> generator) {
        return new AllianceSpecific<>(generator.apply(Alliance.Red), generator.apply(Alliance.Blue));
    }

    public static class AllianceSpecificBuilder<ValueT> {
        private final ValueT valueforRed;

        private AllianceSpecificBuilder(ValueT valueForRed) {
            this.valueforRed = valueForRed;
        }

        public AllianceSpecific<ValueT> forBlue(ValueT valueForBlue) {
            return new AllianceSpecific<>(this.valueforRed, valueForBlue);
        }
    }
}
