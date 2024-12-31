package redbacks.lib.hardware.sensor.ctre;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import redbacks.lib.hardware.sensor.SmartAngleSensor;

public class SmartCancoder {
    private static final Measure<Time>
        CONFIG_TIMEOUT = Units.Milliseconds.of(50),
        STATUS_FRAME_PERIOD = Units.Milliseconds.of(250);

    private final CANcoder cancoder;

    public SmartCancoder(CANcoder cancoder, Rotation2d offset) {
        this.cancoder = cancoder;

        MagnetSensorConfigs config = new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withMagnetOffset(offset.getRotations());

        retryOnError(() -> cancoder.getConfigurator().apply(config, CONFIG_TIMEOUT.in(Units.Seconds)), createLabel("Range & Offset"));
        retryOnError(() -> cancoder.getAbsolutePosition().setUpdateFrequency(1 / STATUS_FRAME_PERIOD.in(Units.Seconds), CONFIG_TIMEOUT.in(Units.Seconds)), createLabel("Update Frequency - Position"));
        retryOnError(() -> cancoder.optimizeBusUtilization(CONFIG_TIMEOUT.in(Units.Seconds)), createLabel("Bus Utilisation Optimisation"));
    }

    public SmartAngleSensor createAngleSensor() {
        return new SmartAngleSensor() {
            @Override
            public Rotation2d getAngle() {
                return Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
            }
        };
    }

    private String createLabel(String task) {
        return "CANCoder " + cancoder.getDeviceID() + " [" + task + "]";
    }

    private void retryOnError(Supplier<StatusCode> configure, String label) {
        StatusCode result = configure.get();

        while(result != StatusCode.OK) {
            System.err.println("Configuration of " + label + " failed with result '" + result + "', retrying in 100ms...");
            Timer.delay(0.1);
            result = configure.get();
        }
    }
}
