package arachne4.lib;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;
import arachne4.lib.subsystems.Subsystem;

public class TestRobot extends Subsystem {
    public static void main(String[] args) {
        ArachneRobot4.startArachneRobot(TestRobot::new, TestRobot.class);
    }

    public TestRobot(SchedulerProvider schedulerProvider) {
        super(schedulerProvider);
    }

    { registerHandler(Scheduler.EXECUTE, () -> {
    }); }
}
