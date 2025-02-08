package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

public interface RobotModule {
    public void Initialize();
    public void ProcessState(boolean value);
    public void SetController(ModuleController controller);
    public String GetModuleID();
    public void ApproachTarget(Pose3d TargetPose);
    public Pose3d GetPosition();
}
