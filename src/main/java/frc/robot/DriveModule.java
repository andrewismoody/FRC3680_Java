package frc.robot;

public interface DriveModule {
    public void Initialize();
    public void ProcessState();
    public void SetController(ModuleController controller);
    public void ProcessForwardSpeed(double value);  
    public void ProcessLateralSpeed(double value);
    public void ProcessRotationAngle(double value);
}
