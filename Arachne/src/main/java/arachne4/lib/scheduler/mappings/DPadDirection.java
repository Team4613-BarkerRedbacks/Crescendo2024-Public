package arachne4.lib.scheduler.mappings;

public enum DPadDirection {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    public final int angle;

    private DPadDirection(int angle) {
        this.angle = angle;
    }

    public static DPadDirection fromAngle(int angle) {
        return switch (angle) {
            case 0 -> UP;
            case 90 -> RIGHT;
            case 180 -> DOWN;
            case 270 -> LEFT;
            default -> null;
        };
    }
}
