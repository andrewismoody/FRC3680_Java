package frc.robot.z2024.action;

// Position is a 'helper' that names positions of a mechanism for easier reading instead of translating the number to a value in your brain
public enum Position {
    // Any should always be -1, which is the 'wildcard' value and matches any Position
    Any(-1),
    Ground(0),
    Lower(1),
    Upper(2),
    Processor(3),
    None(4);

    private final int value;

    private Position(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
