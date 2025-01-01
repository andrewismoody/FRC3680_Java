package frc.robot;

import java.util.Hashtable;

import edu.wpi.first.math.geometry.Translation3d;

public class ModuleController {
  Hashtable<String, RobotModule> modules = new Hashtable<String, RobotModule>();
  DriveModule driveModule;

  double divider = 0.5;
  double speedMod = 1.0;

  private double inverseValue = 1.0;
  private double dividerValue = 1.0;
  private boolean speedLock = false;
  private double speedDilation = 0.0;

  public ModuleController(DriveModule DriveModule, double Divider) {
    driveModule = DriveModule;
    driveModule.SetController(this);
    divider = Divider;
  }

  public DriveModule GetDriveModule() {
    return driveModule;
  }

  public void AddModule(RobotModule module) {
    module.SetController(this);
    modules.put(module.GetModuleID(), module);
  }

  public RobotModule GetModule(String label) {
    return modules.get(label);
  }

  public void ProcessDivider1(double value) {
    if (value > 0) {
      dividerValue = divider;
    } else {
      dividerValue = 1.0;
    }
  }

  public void ProcessInverse(boolean pressed) {
    if (pressed) {
      inverseValue = -1.0;
    } else {
      inverseValue = 1.0;
    }
  }

  public void ProcessSpeedLock(double value) {
    if (value > 0) {
      speedLock = true;
    } else {
      speedLock = false;
    }
  }

  public void ProcessSpeedDilation(double value) {
    var sign = value > 0 ? 1.0 : -1.0;
    speedDilation = Math.abs(value) < 0.75 ? value : 0.75 * sign;
  }

  public double ApplyModifiers(double value) {
    return ApplyModifiers(value, true);
  }

  public double ApplyModifiers(double value, boolean affectSpeed) {
    value *= inverseValue;
    var thisSpeedMod = speedMod;

    if (!speedLock && speedDilation < 0)
      // get the inverse of the Y value subtracted from 1 since the axis is reflected along the X axis
      // >= 0 is always full speed, anything less is a fraction of full speed starting at 1 down to 0
      thisSpeedMod = 1 - -speedDilation;

    if (affectSpeed)
      value *= dividerValue * thisSpeedMod;
    
    return value;
  }

  public void ProcessDrive() {
    driveModule.ProcessState();
  }

  public void setSpeedMod(double newSpeed) {
    if (newSpeed > 0.0)
      speedMod = newSpeed;
  }

  public void setInverseValue(double newInverse) {
    inverseValue = newInverse;
  }

  public Translation3d GetPosition() {
    return driveModule.GetPosition();
  }
}
