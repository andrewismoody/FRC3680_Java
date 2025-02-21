package frc.robot.action;

import edu.wpi.first.math.geometry.Pose3d;

public class ActionPose {
    public Action action;
    public int primary;
    public int secondary;
    public Pose3d position;

    public ActionPose(Action Action, int Primary, int Secondary, Pose3d Position) {
        action = Action;
        primary = Primary;
        secondary = Secondary;
        position = Position;
    }

    public ActionPose(Action Action, int Primary, int Secondary) {
        action = Action;
        primary = Primary;
        secondary = Secondary;
    }
}
