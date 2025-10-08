package frc.robot.positioner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;

public interface Positioner {
    public void Initialize();
    public Translation3d GetPosition();
    public void SetRobotOrientation(String name, double yaw, double yawRate, 
    double pitch, double pitchRate, 
    double roll, double rollRate);
    public boolean IsValid();
    public boolean IsHealthy();
    public String GetHealthReason();
    public Translation3d GetOffset();
}
