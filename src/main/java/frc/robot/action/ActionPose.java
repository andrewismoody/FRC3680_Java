package frc.robot.action;

import frc.robot.auto.AutoTarget;

public class ActionPose {
    public Group group;
    public Location location;
    public int locationIndex;
    public Position position;
    public Action action;
    public AutoTarget target;

    public ActionPose(Group Group, Location Location, int LocationIndex, Position Position, Action Action, AutoTarget Target) {
        group = Group;
        location = Location;
        locationIndex = LocationIndex;
        position = Position;
        action = Action;
        target = Target;
    }
}
