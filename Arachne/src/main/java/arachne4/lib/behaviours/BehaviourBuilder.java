package arachne4.lib.behaviours;

public class BehaviourBuilder implements Behaviour {
    private final Runnable onEnter, run, onLeave;

    private BehaviourBuilder(Runnable onEnter, Runnable run, Runnable onLeave) {
        this.onEnter = onEnter;
        this.run = run;
        this.onLeave = onLeave;
    }

    public static BehaviourBuilder create() {
        return new BehaviourBuilder(() -> {}, () -> {}, () -> {});
    }

    @Override
    public void onEnterMode() {
        onEnter.run();
    }

    @Override
    public void run() {
        run.run();
    }

    @Override
    public void onLeaveMode() {
        onLeave.run();
    }

    public BehaviourBuilder replaceOnEnterWith(Runnable onEnter) {
        return new BehaviourBuilder(onEnter, this.run, this.onLeave);
    }

    public BehaviourBuilder replaceRunWith(Runnable run) {
        return new BehaviourBuilder(this.onEnter, run, this.onLeave);
    }

    public BehaviourBuilder replaceOnLeaveWith(Runnable onLeave) {
        return new BehaviourBuilder(this.onEnter, this.run, onLeave);
    }

    public BehaviourBuilder alsoOnEnter(Runnable additionalOnEnter) {
        return new BehaviourBuilder(() -> { this.onEnter.run(); additionalOnEnter.run(); }, this.run, this.onLeave);
    }

    public BehaviourBuilder alsoRun(Runnable additionallyRun) {
        return new BehaviourBuilder(this.onEnter, () -> { this.run.run(); additionallyRun.run(); }, this.onLeave);
    }

    public BehaviourBuilder alsoOnLeave(Runnable additionalOnLeave) {
        return new BehaviourBuilder(this.onEnter, this.run, () -> { this.onLeave.run(); additionalOnLeave.run(); });
    }
}
