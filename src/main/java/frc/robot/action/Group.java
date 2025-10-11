package frc.robot.action;

public enum Group {
    Stage(0),
    Score(1),
    Pickup(2),
    Start(3),
    Travel(4),
    Align(5),
    Idle(5),
    Any(6),
    None(7);

    private final int value;

    private Group(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
