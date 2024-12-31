package redbacks.robot.subsystems.noteHandling;

import static arachne4.lib.sequences.Actionable.*;

import arachne4.lib.Constants;
import arachne4.lib.behaviours.Behaviour;
import arachne4.lib.behaviours.BehaviourManager;
import arachne4.lib.game.GameState;
import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.sequences.Actionable;
import arachne4.lib.sequences.Untilable;
import arachne4.lib.subsystems.Subsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redbacks.robot.Robot2024;
import redbacks.robot.io.IOProvider;
import redbacks.robot.subsystems.noteHandling.NoteHandlingConstants.IntakeConstants;
import redbacks.robot.subsystems.noteHandling.NoteHandlingConstants.ShooterConstants;
import redbacks.robot.subsystems.noteHandling.pivot.CommonPivot;

public abstract class CommonNoteHandling extends Subsystem {
    private static final ShooterConstants SHOOTER_CONSTANTS = Constants.get(ShooterConstants.class);
    private static final IntakeConstants INTAKE_CONSTANTS = Constants.get(IntakeConstants.class);
    private final CommonNoteHandlingMappings mappings = scheduler.getMappings(CommonNoteHandlingMappings.class);

    public static record ShootingSpeeds(Measure<Velocity<Distance>> speed, double rightMotorSpeedMultiplier) {}

    // ----------------------------------------
    // Behaviours
    // ----------------------------------------

    private final Behaviour shooterIntaking = Behaviour.thatRunsOnce(() -> {
        setShooterSpeed(SHOOTER_CONSTANTS.intakeSpeed());
        setIntakePower(INTAKE_CONSTANTS.outtakePower());
    });

    private final Behaviour sweep = Behaviour.thatRunsOnce(() -> {
        setShooterSpeedWithTopMultiplied(SHOOTER_CONSTANTS.sweepSpeed(), 1);
        setIntakePower(INTAKE_CONSTANTS.power());
    });

    private final Behaviour outtaking = Behaviour.thatRunsOnce(() -> {
        setShooterSpeed(SHOOTER_CONSTANTS.stowSpeed());
        setIntakePower(INTAKE_CONSTANTS.outtakePower());
    });

    private final Behaviour stopIntakeOutake = Behaviour.thatRunsOnce(() -> {
        setShooterSpeed(Units.MetersPerSecond.zero());
        setIntakePower(0);
    });

    private final Behaviour shooterSweepSpinUp = createShooterSweepSpinUpBehaviour();
    protected abstract Behaviour createShooterSweepSpinUpBehaviour();

    private final Behaviour intakeAndShoot = createIntakeAndShootBehaviour();
    protected abstract Behaviour createIntakeAndShootBehaviour();

    // ----- Intaking with sensor -----

    private final Behaviour driverControlled = new Behaviour() {
        public void run() {
            if (mappings.shoot.get()) {
                setShooterToShootingSpeed();
                if(isOnTargetToShootInDriverControlled()) {
                    setIntakePower(INTAKE_CONSTANTS.shootPower());
                }
                else {
                    manageIntakeOperation(INTAKE_CONSTANTS.power());
                }
            }
            else {
                boolean hasNote = manageIntakeOperation(INTAKE_CONSTANTS.power());
                if (hasNote || mappings.accelerateShooter.get()) setShooterToShootingSpeed();
                else setShooterSpeedWithTopMultiplied(SHOOTER_CONSTANTS.stowSpeed(), 1);
            }
        }
    };

    protected abstract boolean isOnTargetToShootInDriverControlled();

    private final Behaviour intaking = automaticIntaking(INTAKE_CONSTANTS.power(), false);
    private final Behaviour intakingWithShooterAtSpeed = automaticIntaking(INTAKE_CONSTANTS.power(), true);
    private final Behaviour shooterSpinUp = automaticIntaking(INTAKE_CONSTANTS.shootPower(), true);

    private Behaviour automaticIntaking(double intakePower, boolean shooterAlwaysAtSpeed) {
        return Behaviour.thatRunsRepeatedly(() -> {
            boolean hasNote = manageIntakeOperation(intakePower);

            if (hasNote || shooterAlwaysAtSpeed) setShooterToShootingSpeed();
            else setShooterSpeedWithTopMultiplied(SHOOTER_CONSTANTS.stowSpeed(), 1);
        });
    }

    private boolean manageIntakeOperation(double intakePower) {
        if (canStowSensorsSeeNote()) {
            setIntakePower(0);
            lastSeenNoteByStowSensorTimeSeconds = Timer.getFPGATimestamp();
            lastSeenNoteByIntakeSensorTimeSeconds = Timer.getFPGATimestamp();
            return true;
        }
        else if (didStowSensorsSeeNoteInOvershootWindow()) {
            setIntakePower(INTAKE_CONSTANTS.reverseAfterStowOvershootPower());
            lastSeenNoteByIntakeSensorTimeSeconds = Timer.getFPGATimestamp();
            return true;
        }
        else if (canIntakeSensorSeeNote()) {
            setIntakePower(INTAKE_CONSTANTS.slowPower());
            lastSeenNoteByIntakeSensorTimeSeconds = Timer.getFPGATimestamp();
            return false;
        }
        else if (didIntakeSensorsSeeNoteInSlowWindow()) {
            setIntakePower(INTAKE_CONSTANTS.slowPower());
            return false;
        }
        else {
            setIntakePower(intakePower);
            return false;
        }
    }

    private void setShooterToShootingSpeed() {
        ShootingSpeeds shootingSpeeds = determineShootingSpeeds();
        setShooterSpeedWithTopMultiplied(shootingSpeeds.speed, shootingSpeeds.rightMotorSpeedMultiplier);
    }

    // ----- Shooting -----

    private final Behaviour shooting = new Behaviour() {
        @Override
        public void onEnterMode() {
            setShooterSpeed(SHOOTER_CONSTANTS.shootSpeed());
        }

        @Override
        public void run() {
            setIntakePower(isOnTargetToShootInAuto() ? INTAKE_CONSTANTS.shootPower() : 0);
        }
    };

    protected abstract boolean isOnTargetToShootInAuto();

    // ----------------------------------------
    // Subsystem Definition
    // ----------------------------------------

    protected final NoteHandlingIO io;

    protected final CommonPivot pivot;
    protected final Robot2024 robot;

    protected final BehaviourManager<Behaviour> handlingBehaviours;

    private double lastSeenNoteByStowSensorTimeSeconds = Double.NaN;
    private double lastSeenNoteByIntakeSensorTimeSeconds = Double.NaN;

    public CommonNoteHandling(Robot2024 robot, CommonPivot pivot, IOProvider io) {
        super(robot);
        this.robot = robot;

        this.io = io.getNoteHandlingIO();

        this.pivot = pivot;

        this.handlingBehaviours = new BehaviourManager<Behaviour>(robot, driverControlled);
        registerHandler(Scheduler.EXECUTE, handlingBehaviours);

        mappings.registerRumble(this::canIntakeSensorSeeNote);
    }

    // ----------------------------------------
    // Subsystem Operations
    // ----------------------------------------

    protected abstract ShootingSpeeds determineShootingSpeeds();

    // ----- Intake -----

    public boolean canStowSensorsSeeNote() {
        return io.inputs.isNoteInStow;
    }

    private boolean canIntakeSensorSeeNote() {
        return io.inputs.isNoteInIntake;
    }

    private boolean didStowSensorsSeeNoteInOvershootWindow() {
        return Timer.getFPGATimestamp() - lastSeenNoteByStowSensorTimeSeconds < INTAKE_CONSTANTS.stowReverseAfterOvershootDuration().in(Units.Seconds);
    }

    private boolean didIntakeSensorsSeeNoteInSlowWindow() {
        return Timer.getFPGATimestamp() - lastSeenNoteByIntakeSensorTimeSeconds < INTAKE_CONSTANTS.slowIntakeAfterDetectionDuration().in(Units.Seconds);
    }

    protected void setIntakePower(double power) {
        io.setIntakePercentageOutput(power);
    }

    // ----- Shooter -----

    protected void setShooterSpeed(Measure<Velocity<Distance>> speed) {
        setShooterSpeedWithTopMultiplied(speed, SHOOTER_CONSTANTS.generalRightRollerSpeedMultiplier());
    }

    protected void setShooterSpeedWithTopMultiplied(Measure<Velocity<Distance>> speed, double multiplier) {
        io.setRightShooterTargetVelocity(speed);
        io.setLeftShooterTargetVelocity(speed.times(multiplier));
    }

    // ----------------------------------------
    // Public State-Modifiers & Accessors
    // ----------------------------------------

    public boolean hasNote() {
        return canStowSensorsSeeNote() || didStowSensorsSeeNoteInOvershootWindow();
    }

    public abstract boolean isShooterAtSpeakerSpeed();

    public Rotation2d getPivotAngle() {
        return pivot.getAngle();
    }

    // ----- Handling behaviours -----

    public void intake() {
        handlingBehaviours.changeToMode(intaking);
    }

    public void intakeWithShooterAtSpeed() {
        handlingBehaviours.changeToMode(intakingWithShooterAtSpeed);
    }

    public void shoot() {
        handlingBehaviours.changeToMode(shooting);
    }

    public void stopIntakeOutake() {
        handlingBehaviours.changeToMode(stopIntakeOutake);
    }

    public void sweep() {
        handlingBehaviours.changeToMode(sweep);
    }

    public void shooterSpinUp() {
        handlingBehaviours.changeToMode(shooterSpinUp);
    }

    public void shooterSweepSpinUp() {
        handlingBehaviours.changeToMode(shooterSweepSpinUp);
    }

    public void intakeShoot() {
        handlingBehaviours.changeToMode(intakeAndShoot);
    }

    protected Measure<Velocity<Distance>> currentSpeed() {
        return io.inputs.rightShooterVelocity;
    }

    // ----- Aiming behaviours -----

    public void moveToAngle(Rotation2d angle) {
        pivot.moveTo(angle);
    }

    // ----- Actionables -----

    public abstract Actionable doShoot();

    public Untilable waitForNotePassthrough(Measure<Time> maxWaitTime) {
        return SEQUENCE(
            WAIT(maxWaitTime).UNSAFE_UNTIL(waitTimeElapsed -> waitTimeElapsed || io.inputs.isNoteInStow),
            WAIT().UNSAFE_UNTIL(() -> !io.inputs.isNoteInStow),
            WAIT(Units.Seconds.of(0.1))
        );
    }

    // ----------------------------------------
    // Scheduled Events
    // ----------------------------------------

    { registerHandler(Scheduler.EXECUTE, this::telemetry); }
    private void telemetry() {
        SmartDashboard.putBoolean("Stow Beam", canStowSensorsSeeNote());
        SmartDashboard.putBoolean("Intake Beam", canIntakeSensorSeeNote());
        SmartDashboard.putBoolean("Has Note", hasNote());
        SmartDashboard.putNumber("Shooter Right Power", io.inputs.rightShooterVelocity.in(Units.MetersPerSecond));
        SmartDashboard.putNumber("Shooter Left Power", io.inputs.leftShooterVelocity.in(Units.MetersPerSecond));
        SmartDashboard.putNumber("Shooter Set Speed", determineShootingSpeeds().speed.in(Units.MetersPerSecond));
    }

    // ----- Controls -----

    { registerHandler(Scheduler.EXECUTE, GameState.DRIVER_CONTROLLED_STATES, this::manageHandlingModes); }
    private void manageHandlingModes() {
        if (mappings.outtake.get()) {
            handlingBehaviours.changeToModeIfNotAlreadyIn(outtaking);
        }
        else if (mappings.shooterIntake.get()) {
            handlingBehaviours.changeToModeIfNotAlreadyIn(shooterIntaking);
        }
        else {
            handlingBehaviours.changeToModeIfNotAlreadyIn(driverControlled);
        }
    }
}
