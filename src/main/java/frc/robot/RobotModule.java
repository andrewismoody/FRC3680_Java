package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;

public interface RobotModule {
    public void Initialize();
    public void ProcessState(boolean value);
    public void ApplyValue(boolean value);
    public void ApplyInverse(boolean value);
    public void SetController(ModuleController controller);
    public String GetModuleID();
    public void SetTargetActionPose(Action action, int primary, int secondary);
    public Pose3d GetPosition();
    public void AddActionPose(ActionPose NewPose);
    public ActionPose GetActionPose(Action action, int Primary, int Secondary);
}
