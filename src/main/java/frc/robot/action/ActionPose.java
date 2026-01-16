package frc.robot.action;

import frc.robot.auto.AutoTarget;

public class ActionPose {
    public String group;
    public String location;
    public int locationIndex;
    public String position;
    public String action;
    public AutoTarget target;

    public ActionPose(String Group, String Location, int LocationIndex, String Position, String Action, AutoTarget Target) {
        group = Group;
        location = Location;
        locationIndex = LocationIndex;
        position = Position;
        action = Action;
        target = Target;
    }
}
