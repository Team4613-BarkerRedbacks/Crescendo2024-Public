package redbacks.robot.subsystems.drivetrain.behaviours;

import arachne4.lib.behaviours.Behaviour;
import redbacks.robot.subsystems.drivetrain.DrivetrainIO;

public class DrivetrainBehaviour implements Behaviour {
    protected final DrivetrainIO io;

    protected DrivetrainBehaviour(DrivetrainIO io) {
        this.io = io;
    }

    public void acceptDriverInputs(double forward, double left, double rotate) {}
}
