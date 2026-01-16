package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.action.ActionPose;
public interface RobotModule {
    public void Initialize();
    public void ProcessState(boolean value);
    public void ApplyValue(boolean value);
    public void ApplyInverse(boolean value);
    public void SetController(ModuleController controller);
    public String GetModuleID();
    public void SetTargetActionPose(String Group, String Location, int LocationIndex, String Position, String Action);
    public void SetTargetActionPose(ActionPose ActionPose);
    public Pose3d GetPosition();
    public void AddActionPose(ActionPose NewPose);
    public ActionPose GetActionPose(String Group, String Location, int LocationIndex, String Position, String Action);
    public ActionPose GetTarget();
    public void AbandonTarget();
}
