package arachne4.lib.subsystems;

import arachne4.lib.scheduler.Scheduler;
import arachne4.lib.scheduler.SchedulerProvider;

public class Subsystem implements SchedulerProvider {
    protected final Scheduler scheduler;

    public Subsystem(SchedulerProvider schedulerProvider) {
        this.scheduler = schedulerProvider.getScheduler();
    }

    @Override
    public Scheduler getScheduler() {
        return scheduler;
    }

    public static class SubsystemWithRobotReference<RobotT extends SchedulerProvider> extends Subsystem {
        protected final RobotT robot;

        public SubsystemWithRobotReference(RobotT robot) {
            super(robot);
            this.robot = robot;
        }
    }

    public static class SubsystemWithParent<RobotT extends SchedulerProvider, ParentT extends Subsystem> extends SubsystemWithRobotReference<RobotT> {
        protected final ParentT parent;

        public SubsystemWithParent(RobotT robot, ParentT parent) {
            super(robot);
            this.parent = parent;
        }
    }
}
