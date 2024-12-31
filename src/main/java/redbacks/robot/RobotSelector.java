package redbacks.robot;

import static arachne4.lib.ArachneRobotWithAdvantageKit.startArachneRobotWithAdvantageKit;

import arachne4.lib.ArachneRobotWithAdvantageKit.Mode;
import edu.wpi.first.wpilibj.RobotBase;
import redbacks.robot.scorpion.Scorpion;

public class RobotSelector {
    public static void main(String[] args) {
        startScorpion(RobotBase.isSimulation() ? Mode.SIMULATED : Mode.REAL);
    }

    private static void startScorpion(Mode mode) {
        startArachneRobotWithAdvantageKit(Scorpion::new, Scorpion.class, CommonConstants.LOOP_PERIOD, mode);
    }
}
