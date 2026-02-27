package frc.robot.z2026.action;

// Location is a 'helper' that names locations on the field for easier reading instead of translating the number to a value in your brain
public enum Location {
    // Any should always be -1, which is the 'wildcard' value and matches any Location
    Any(-1),
    Start(0),
    Hub(1),
    Depot(2),
    Outpost(3),
    Tower(4),
    Trench(5),
    Bump(6),
    AdHoc(7),
    Waypoint(8),
    Tag(9),
    Interest(10),
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
