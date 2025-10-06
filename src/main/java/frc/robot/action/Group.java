package frc.robot.action;

public enum Group {
    Stage(0),
    Score(1),
    Pickup(2),
    Start(3),
    Idle(4),
    Any(5),
    None(6);

    private final int value;

    private Group(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
