package arachne4.lib.behaviours;

public interface Behaviour {
    default void onEnterMode() {}
    default void run() {}
    default void onLeaveMode() {}

    public static Behaviour thatRunsOnce(Runnable onEnterMode) {
        return new Behaviour() {
            @Override
            public void onEnterMode() {
                onEnterMode.run();
            }
        };
    }

    public static Behaviour thatRunsRepeatedly(Runnable run) {
        return new Behaviour() {
            @Override
            public void run() {
                run.run();
            }
        };
    }
}
