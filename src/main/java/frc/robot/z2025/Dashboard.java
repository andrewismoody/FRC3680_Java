package frc.robot.z2025;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.action.Action;
import frc.robot.action.Group;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;

public class Dashboard {
  public static void InitializeChoosers() {
    SendableChooser<Group> GroupChooser = new SendableChooser<>();
    for (Group item : Group.class.getEnumConstants()) {
      GroupChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(GroupChooser);

    SendableChooser<Location> LocationChooser = new SendableChooser<>();
    for (Location item : Location.class.getEnumConstants()) {
      LocationChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(LocationChooser);

    SendableChooser<Integer> IndexChooser = new SendableChooser<>();
    for (int i = 0; i < 8; i++) {
      IndexChooser.addOption(String.valueOf(i), i);
    }
    SmartDashboard.putData(IndexChooser);

    SendableChooser<Position> PositionChooser = new SendableChooser<>();
    for (Position item : Position.class.getEnumConstants()) {
      PositionChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(PositionChooser);

    SendableChooser<Action> ActionChooser = new SendableChooser<>();
    for (Action item : Action.class.getEnumConstants()) {
      ActionChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(ActionChooser);
  }    
}
