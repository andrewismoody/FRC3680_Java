package frc.robot;

interface RobotModule {
    public void Initialize();
    public void ProcessState(boolean value);
    public void SetController(ModuleController controller);
}
