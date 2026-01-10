package frc.robot.zMechinarium.action;

// Location is a 'helper' that names locations on the field for easier reading instead of translating the number to a value in your brain
public enum Location {
    // Any should always be -1, which is the 'wildcard' value and matches any Location
    Any(-1),
    Reef(0),
    Processor(1),
    Coral(2),
    Barge(3),
    AdHoc(4),
    Waypoint(5),
    Tag(6),
    Interest(7),
    None(11),
    ;

    private final int value;

    private Location(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }
}
