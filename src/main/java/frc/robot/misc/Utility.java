package frc.robot.misc;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utility {

  public static boolean IsRedAlliance() {
      return DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() != Alliance.Blue;
  }

}
