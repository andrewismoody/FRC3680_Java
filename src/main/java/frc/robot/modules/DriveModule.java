package frc.robot.modules;

import edu.wpi.first.math.geometry.Pose3d;

public interface DriveModule extends RobotModule {
    public void Initialize();
    public void SetController(ModuleController controller);
    public void ProcessForwardSpeed(double value);  
    public void ProcessLateralSpeed(double value);
    public void ProcessRotationAngle(double value);
    public void StopRotation();
    public void SetFieldOriented(boolean value);
    public boolean IsFieldOriented();
    public void ToggleFieldOriented(boolean value);
    public void SetCurrentPose(Pose3d newPose);
    public Pose3d GetPositionerOffset();
    public void ResetGyro();
    public void ResetEncoders();
}
