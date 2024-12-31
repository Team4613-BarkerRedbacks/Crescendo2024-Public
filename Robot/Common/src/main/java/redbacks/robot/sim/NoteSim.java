package redbacks.robot.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import redbacks.robot.CommonConstants;

public class NoteSim {
    private static final Measure<Distance>
        MINOR_RADIUS = Units.Inches.of(1),
        MAJOR_RADIUS = Units.Inches.of(6),
        TOTAL_RADIUS = MAJOR_RADIUS.plus(MINOR_RADIUS);

    private static final Measure<Distance>
        AMP_HEIGHT = Units.Feet.of(2 + 1).plus(Units.Inches.of(2 + 6)),
        TRAP_HEIGHT = Units.Feet.of(4 + 1).plus(Units.Inches.of(8.5 + 6)),
        BOTTOM_OF_TRAP_HEIGHT = Units.Feet.of(2).plus(Units.Inches.of(4.25 + 2));

    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.8);

    private Pose3d currentPose;
    private boolean isAirborne = false, isInTrap = false, isInAmp = false;
    private Translation3d metersPerSecond = new Translation3d();

    public NoteSim(Translation2d stagedLocation) {
        this(getGroundedPose(stagedLocation));
    }

    public NoteSim(Pose3d initialPose) {
        this.currentPose = initialPose;
    }

    private static Pose3d getGroundedPose(Translation2d fieldCoord) {
        return new Pose3d(fieldCoord.getX(), fieldCoord.getY(), MINOR_RADIUS.in(Units.Meters), new Rotation3d());
    }

    private static Pose3d getUprightPose(Translation2d fieldCoord, Rotation2d rotation, Measure<Distance> height) {
        return new Pose3d(
            fieldCoord.getX(),
            fieldCoord.getY(),
            height.in(Units.Meters),
            new Rotation3d(0, Math.PI / 2, rotation.getRadians()));
    }

    public void update() {
        if (isAirborne) updateWhenAirborne();
        else if (metersPerSecond.getNorm() > 0.01) updateWhenSliding();
    }

    private void updateWhenAirborne() {
        double dtSeconds = CommonConstants.LOOP_PERIOD.in(Units.Seconds);
        metersPerSecond = metersPerSecond
            .plus(GRAVITY.times(dtSeconds))
            .times(isInAmp || isInTrap ? 0.95 : 0.99); // Drag

        currentPose = currentPose.plus(new Transform3d(metersPerSecond.times(dtSeconds).rotateBy(currentPose.getRotation().unaryMinus()), new Rotation3d()));

        if (isInAmp) {
            if (currentPose.getZ() - TOTAL_RADIUS.in(Units.Meters) < 0) {
                currentPose = getUprightPose(
                    currentPose.getTranslation().toTranslation2d(),
                    currentPose.getRotation().toRotation2d(),
                    TOTAL_RADIUS);
                metersPerSecond = new Translation3d();
                isAirborne = false;
            }
        }
        else if (isInTrap) {
            if (currentPose.getZ() - TOTAL_RADIUS.in(Units.Meters) < BOTTOM_OF_TRAP_HEIGHT.in(Units.Meters)) {
                currentPose = getUprightPose(
                    currentPose.getTranslation().toTranslation2d(),
                    currentPose.getRotation().toRotation2d(),
                    BOTTOM_OF_TRAP_HEIGHT.plus(TOTAL_RADIUS));
                metersPerSecond = new Translation3d();
                isAirborne = false;
            }
        }
        else if (currentPose.getZ() < MINOR_RADIUS.in(Units.Meters)) {
            currentPose = getGroundedPose(currentPose.getTranslation().toTranslation2d());
            metersPerSecond = new Translation3d(metersPerSecond.getX(), metersPerSecond.getY(), 0);
            isAirborne = false;
        }
    }

    private void updateWhenSliding() {
        double dtSeconds = CommonConstants.LOOP_PERIOD.in(Units.Seconds);
        metersPerSecond = metersPerSecond.times(0.95); // Drag
        currentPose = currentPose.plus(new Transform3d(metersPerSecond.times(dtSeconds).rotateBy(currentPose.getRotation().unaryMinus()), new Rotation3d()));
    }

    public Pose3d getCurrentPose() {
        return currentPose;
    }

    public boolean canBePickedUp() {
        return !isAirborne && !isInAmp && !isInTrap;
    }

    public void setPose(Pose3d pose) {
        this.currentPose = pose;
    }

    public void launch(Rotation2d direction, ChassisSpeeds fieldRelativeSpeed, double metersPerSecond, Rotation2d launchAngle) {
        if (metersPerSecond > 5) metersPerSecond = (metersPerSecond - 5) * 0.7 + 5; // Loss to inefficient transfer of velocities

        double vx = metersPerSecond * direction.getCos() * launchAngle.getCos() + fieldRelativeSpeed.vxMetersPerSecond;
        double vy = metersPerSecond * direction.getSin() * launchAngle.getCos() + fieldRelativeSpeed.vyMetersPerSecond;
        double vz = metersPerSecond * launchAngle.getSin();

        this.isAirborne = true;
        this.metersPerSecond = new Translation3d(vx, vy, vz);
    }
}
