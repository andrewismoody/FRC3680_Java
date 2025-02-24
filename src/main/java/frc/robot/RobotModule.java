package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.action.*;

public interface RobotModule {
    public void Initialize();
    public void ProcessState(boolean value);
    public void ApplyValue(boolean value);
    public void ApplyInverse(boolean value);
    public void SetController(ModuleController controller);
    public String GetModuleID();
    public void SetTargetActionPose(Group Group, Location Location, int LocationIndex, Position Position, Action Action);
    public void SetTargetActionPose(ActionPose ActionPose);
    public Pose3d GetPosition();
    public void AddActionPose(ActionPose NewPose);
    public ActionPose GetActionPose(Group Group, Location Location, int LocationIndex, Position Position, Action Action);
}
