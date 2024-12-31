package redbacks.lib.math.characteristics;

public class GearRatio {
    private final double ratio;

    public GearRatio(double in, double out) {
        this.ratio = in / out;
    }

    public double getOutputCountFromInput(double input) {
        return input / ratio;
    }

    public double getInputCountFromOutput(double output) {
        return output * ratio;
    }

    public double getReduction() {
        return ratio;
    }
}
