package frc.robot.action;

public enum Group {
    Stage(0),
    Score(1),
    Pickup(2),
    Start(3),
    Travel(4),
    AlignLeft(5),
    AlignRight(6),
    Align(7),
    ApproachLeft(8),
    ApproachRight(9),
    Approach(10),
    Idle(11),
    Any(12),
    None(13);

    private final int value;

    private Group(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
