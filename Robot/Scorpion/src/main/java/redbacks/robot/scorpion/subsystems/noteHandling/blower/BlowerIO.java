package redbacks.robot.scorpion.subsystems.noteHandling.blower;

import redbacks.lib.subsystems.positionbased.AngleBasedIO;

public interface BlowerIO extends AngleBasedIO {
    String KEY = "Blower";

    void blowWithPercentageOutput(double power);

    public static class Empty extends AngleBasedIO.Empty implements BlowerIO {
        @Override public void blowWithPercentageOutput(double power) {}
    }
}
