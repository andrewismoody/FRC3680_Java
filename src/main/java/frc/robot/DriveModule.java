package frc.robot;

public interface DriveModule extends RobotModule {
    public void Initialize();
    public void SetController(ModuleController controller);
    public void ProcessForwardSpeed(double value);  
    public void ProcessLateralSpeed(double value);
    public void ProcessRotationAngle(double value);
    public void StopRotation();
}
