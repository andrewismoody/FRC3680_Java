package frc.robot.positioner;

import edu.wpi.first.math.geometry.Pose3d;

public interface Positioner {
    public void Initialize();
    public Pose3d GetPose();
    public void SetRobotOrientation(String name, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate);
    public boolean IsValid();
    public boolean IsHealthy();
    public String GetHealthReason();
    public Pose3d GetReferenceInFieldCoords();
    public Pose3d GetReferenceInRobotCoords();
}
