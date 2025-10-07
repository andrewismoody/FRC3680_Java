package frc.robot.modules;

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
}
