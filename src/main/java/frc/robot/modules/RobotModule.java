package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.action.Group;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
public interface RobotModule {
    public void Initialize();
    public void ProcessState(boolean value);
    public void ApplyValue(boolean value);
    public void ApplyInverse(boolean value);
    public void SetController(ModuleController controller);
    public String GetModuleID();
    public void SetTargetActionPose(Group Group, int Location, int LocationIndex, int Position, Action Action);
    public void SetTargetActionPose(ActionPose ActionPose);
    public Pose3d GetPosition();
    public void AddActionPose(ActionPose NewPose);
    public ActionPose GetActionPose(Group Group, int Location, int LocationIndex, int Position, Action Action);
    public ActionPose GetTarget();
    public void AbandonTarget();
}
