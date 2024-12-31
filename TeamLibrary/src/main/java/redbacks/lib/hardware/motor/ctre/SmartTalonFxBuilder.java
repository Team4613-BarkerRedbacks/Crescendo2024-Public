package redbacks.lib.hardware.motor.ctre;

import java.util.LinkedList;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Pair;

public class SmartTalonFxBuilder {
    private final TalonFX primaryMotor;
    private final NeutralModeValue neutralMode;

    private final List<Pair<TalonFX, Boolean>> followers = new LinkedList<>();

    private TalonFXConfiguration config = new TalonFXConfiguration();
    private boolean useFieldOrientedControl = true;

    private SmartTalonFxBuilder(TalonFX primaryMotor, NeutralModeValue neutralMode) {
        this.primaryMotor = primaryMotor;
        this.neutralMode = neutralMode;
    }

    public static final SmartTalonFxBuilder fromMotor(TalonFX primaryMotor, NeutralModeValue neutralMode) {
        return new SmartTalonFxBuilder(primaryMotor, neutralMode);
    }

    public static final SmartTalonFxBuilder fromId(int id, NeutralModeValue neutralMode) {
        return fromMotor(new TalonFX(id), neutralMode);
    }

    public static final SmartTalonFxBuilder fromId(int id, String busName, NeutralModeValue neutralMode) {
        return fromMotor(new TalonFX(id, busName), neutralMode);
    }

    public final SmartTalonFxBuilder withConfig(TalonFXConfiguration config) {
        this.config = config;

        return this;
    }

    public final SmartTalonFxBuilder disableFieldOrientedControl() {
        useFieldOrientedControl = false;

        return this;
    }

    public final SmartTalonFxBuilder addMotor(TalonFX follower, boolean opposeMaster) {
        this.followers.add(Pair.of(follower, opposeMaster));

        return this;
    }

    public final SmartTalonFx buildUsingFollowers() {
        var motors = new LinkedList<TalonFX>();

        configureMotor(primaryMotor, config, neutralMode);
        motors.add(primaryMotor);

        for(var motorAndNeutralModeAndShouldOppose : followers) {
            var follower = motorAndNeutralModeAndShouldOppose.getFirst();
            var shouldOppose = motorAndNeutralModeAndShouldOppose.getSecond();

            configureMotor(follower, config, neutralMode);

            follower.setControl(new Follower(primaryMotor.getDeviceID(), shouldOppose));

            motors.add(follower);
        }

        return new SmartTalonFx(new TalonFxMotorGroupWithFollowers(primaryMotor, motors), useFieldOrientedControl);
    }

    public final SmartTalonFx buildUsingIndividualControl() {
        var motors = new LinkedList<TalonFX>();

        configureMotor(primaryMotor, config, neutralMode);
        motors.add(primaryMotor);

        for(var motorAndNeutralModeAndShouldOppose : followers) {
            var follower = motorAndNeutralModeAndShouldOppose.getFirst();
            var shouldOppose = motorAndNeutralModeAndShouldOppose.getSecond();

            configureMotor(follower, config, NeutralModeValue.Coast);
            follower.setInverted((primaryMotor.getInverted() && !shouldOppose) || (!primaryMotor.getInverted() && shouldOppose));

            motors.add(follower);
        }

        return new SmartTalonFx(new TalonFxMotorGroupIndividualControl(motors), useFieldOrientedControl);
    }

    private static final void configureMotor(TalonFX motor, TalonFXConfiguration config, NeutralModeValue neutralMode) {
        config.MotorOutput.NeutralMode = neutralMode;
        motor.getConfigurator().apply(config);
    }
}
