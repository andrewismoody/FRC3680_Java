package frc.robot.action;

import frc.robot.auto.AutoTarget;

public class ActionPose {
    public Group group;
    public int location;
    public int locationIndex;
    public int position;
    public Action action;
    public AutoTarget target;

    public ActionPose(Group Group, int Location, int LocationIndex, int Position, Action Action, AutoTarget Target) {
        group = Group;
        location = Location;
        locationIndex = LocationIndex;
        position = Position;
        action = Action;
        target = Target;
    }
}
