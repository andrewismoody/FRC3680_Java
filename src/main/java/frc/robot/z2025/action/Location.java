package frc.robot.z2025.action;

public enum Location {
    // Any should always be -1, which is the 'wildcard' value and matches any Location
    Any(-1),
    Reef(0),
    Processor(1),
    Coral(2),
    Barge(3),
    AdHoc(4),
    Waypoint(5),
    None(6);

    private final int value;

    private Location(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
