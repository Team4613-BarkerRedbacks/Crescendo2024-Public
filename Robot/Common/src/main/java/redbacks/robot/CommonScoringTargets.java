package redbacks.robot;

public class CommonScoringTargets {
    public static class ScoringTarget {
        private final String name;

        public ScoringTarget(String name) {
            this.name = name;
        }

        @Override
        public String toString() {
            return name;
        }
    }

    public static final ScoringTarget
        SPEAKER = new ScoringTarget("Speaker"),
        AMP = new ScoringTarget("Amp"),
        FEED_TO_CENTER = new ScoringTarget("Feed to Center"),
        FEED_TO_WING = new ScoringTarget("Feed to Wing");
}
