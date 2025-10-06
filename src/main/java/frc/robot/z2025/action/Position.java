package frc.robot.z2025.action;

public enum Position {
    // Any should always be -1, which is the 'wildcard' value and matches any Position
    Any(-1),
    Ground(0),
    Trough(1),
    Lower(2),
    Middle(3),
    Upper(4),
    Processor(5),
    None(7);

    private final int value;

    private Position(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
