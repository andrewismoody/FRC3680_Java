package frc.robot.z2024;

import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.misc.GameController;
import frc.robot.misc.GameController.ButtonName;
import frc.robot.modules.DifferentialDriveModule;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleMotorModule;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;

public class Joystick {
  public static void InitializeButtonMappings(GameController m_controller,
    ModuleController modules, DifferentialDriveModule diffDriveModule,
    SingleMotorModule shoot, SingleMotorModule feed, SingleMotorModule pickup) {
    // TODO: need to move button mappings to preferences and initialize in game controller class
    
    m_controller.ClearAllRegistrations();

    // operate feed and pickup simulataneously
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, feed::ApplyValue);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, pickup::ApplyValue);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, feed::ApplyInverse);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, pickup::ApplyInverse);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.BottomButton, shoot::SetNoPose);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, shoot.AddButtonMappedPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, null)));
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, shoot.AddButtonMappedPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, null)));
    m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, shoot.AddButtonMappedPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, null)));

    m_controller.RegisterBinaryButtonConsumer(ButtonName.POVUp, shoot::ApplyValue);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.POVDown, shoot::ApplyInverse);

    // m_controller.RegisterBinaryButtonConsumer(ButtonName.POVLeft, grabber::ApplyInverse);
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.POVRight, grabber::ApplyValue);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.Start, modules.GetDriveModule()::ToggleFieldOriented);

    m_controller.RegisterValueButtonConsumer(ButtonName.LeftTrigger, modules::ProcessDivider1);

    m_controller.SetValueButtonInversion(ButtonName.RightTrigger, true);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightTrigger, modules::ProcessSpeedDilation);

    m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickY, true);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickY, diffDriveModule::ProcessForwardSpeed);

    m_controller.SetValueButtonInversion(ButtonName.RightThumbstickX, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickX, diffDriveModule::ProcessRotationAngle);
  }
}
