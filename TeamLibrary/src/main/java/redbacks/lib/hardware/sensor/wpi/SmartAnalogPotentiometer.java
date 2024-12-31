package redbacks.lib.hardware.sensor.wpi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import redbacks.lib.hardware.sensor.SmartAngleSensor;

public class SmartAnalogPotentiometer implements SmartAngleSensor {
    private final AnalogPotentiometer potentiometer;
    private final AngleMode angleMode;
    private final boolean invert;

    public SmartAnalogPotentiometer(AnalogInput input, AngleMode angleMode, Rotation2d offset, boolean invert) {
        double offsetAdjustedDegrees = invert ? -offset.getDegrees() : offset.getDegrees();

        this.potentiometer = new AnalogPotentiometer(input, angleMode.fullRangeDegrees, angleMode.minAngleDegrees + offsetAdjustedDegrees);
        this.angleMode = angleMode;
        this.invert = invert;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
            MathUtil.inputModulus(invert ? -potentiometer.get() : potentiometer.get(), angleMode.minAngleDegrees, angleMode.maxAngleDegrees)
        );
    }

    public static enum AngleMode {
        FROM_MINUS_180_TO_180(-180, 180),
        FROM_0_TO_360(0, 360);

        private final int minAngleDegrees, maxAngleDegrees, fullRangeDegrees;

        AngleMode(int minAngleDegrees, int maxAngleDegrees) {
            this.minAngleDegrees = minAngleDegrees;
            this.maxAngleDegrees = maxAngleDegrees;
            this.fullRangeDegrees = maxAngleDegrees - minAngleDegrees;
        }
    }
}