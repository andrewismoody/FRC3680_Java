package frc.robot.auto;

import frc.robot.ModuleController;

public interface Auto {
    void Initialize();
    void Update();
    void Shutdown();
    void SetController(ModuleController controller);
    void AddEvent(AutoEvent event);
}
