package frc.robot.z2025;

import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.misc.GameController;
import frc.robot.misc.GameController.ButtonName;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.SwerveDriveModule;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;

public class Joystick {
  static boolean hasRegistered = false;

  public static void InitializeButtonMappings(GameController m_controller,
    ModuleController modules, SwerveDriveModule swerveDriveModule, SingleActuatorModule slide,
    SingleMotorModule elevator) { //, DualMotorModule grabber) {
    // TODO: need to move button mappings to preferences and initialize in game controller class
    
    // prevent duplicate registration
    if (!hasRegistered) {
      hasRegistered = true;

      m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, slide::ApplyValue);
      m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, slide::ApplyInverse);

      m_controller.RegisterBinaryButtonConsumer(ButtonName.BottomButton, elevator::SetNoPose);
      m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, elevator.AddButtonMappedPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, null)));
      m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, elevator.AddButtonMappedPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, null)));
      m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, elevator.AddButtonMappedPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, null)));

      m_controller.RegisterBinaryButtonConsumer(ButtonName.POVDown, elevator::ApplyInverse);
      m_controller.RegisterBinaryButtonConsumer(ButtonName.POVUp, elevator::ApplyValue);

      // m_controller.RegisterBinaryButtonConsumer(ButtonName.POVLeft, grabber::ApplyInverse);
      // m_controller.RegisterBinaryButtonConsumer(ButtonName.POVRight, grabber::ApplyValue);

      // m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, swerveDriveModule::LockPosition);
      m_controller.RegisterBinaryButtonConsumer(ButtonName.Select, swerveDriveModule::ReturnToZero);
      m_controller.RegisterBinaryButtonConsumer(ButtonName.Select, elevator.AddButtonMappedPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, null)));
      m_controller.RegisterBinaryButtonConsumer(ButtonName.Start, modules.GetDriveModule()::ToggleFieldOriented);

      // m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, modules::ProcessInverse);

      m_controller.RegisterValueButtonConsumer(ButtonName.LeftTrigger, modules::ProcessDivider1);

      m_controller.SetValueButtonInversion(ButtonName.RightTrigger, true);
      m_controller.RegisterValueButtonConsumer(ButtonName.RightTrigger, modules::ProcessSpeedDilation);

      m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickY, true);
      m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickY, swerveDriveModule::ProcessForwardSpeed);

      m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickX, true);
      m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickX, swerveDriveModule::ProcessLateralSpeed);

      m_controller.SetValueButtonInversion(ButtonName.RightThumbstickX, false);
      m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickX, swerveDriveModule::ProcessRotationAngle);
    }
  }
}
