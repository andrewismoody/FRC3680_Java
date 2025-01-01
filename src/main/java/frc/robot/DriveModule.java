package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;

public interface DriveModule {
    public void Initialize();
    public void ProcessState();
    public void SetController(ModuleController controller);
    public void ProcessForwardSpeed(double value);  
    public void ProcessLateralSpeed(double value);
    public void ProcessRotationAngle(double value);
    public Translation3d GetPosition();
}
