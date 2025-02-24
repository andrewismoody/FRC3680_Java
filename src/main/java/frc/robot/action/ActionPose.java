package frc.robot.action;

import edu.wpi.first.math.geometry.Pose3d;

public class ActionPose {
    public Group group;
    public Location location;
    public int locationIndex;
    public Position position;
    public Action action;
    public Pose3d pose;

    public ActionPose(Group Group, Location Location, int LocationIndex, Position Position, Action Action, Pose3d Pose) {
        group = Group;
        location = Location;
        locationIndex = LocationIndex;
        position = Position;
        action = Action;
        pose = Pose;
    }
}
