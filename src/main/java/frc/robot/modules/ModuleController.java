package frc.robot.modules;

import java.util.Hashtable;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.misc.GameController;

public class ModuleController {
  Hashtable<String, RobotModule> modules = new Hashtable<String, RobotModule>();
  DriveModule driveModule;
  GameController controller;

  double divider = 0.5;
  double speedMod = 1.0;

  private double inverseValue = 1.0;
  private double dividerValue = 1.0;
  private boolean speedLock = false;
  private double speedDilation = 0.0;

  boolean enableDrive = true;
  boolean enableSteer = true;
  boolean enableDriveTrain = true;
  double speedDilationLimit = 0.75; //0.9;

  public ModuleController(DriveModule DriveModule, double Divider) {
    driveModule = DriveModule;
    driveModule.SetController(this);
    divider = Divider;
  }

  public void Initialize(GameController Controller) {
    controller = Controller;

    for (RobotModule module : modules.values()) {
      module.Initialize();
    }
    
    driveModule.Initialize();
  }

  public DriveModule GetDriveModule() {
    return driveModule;
  }

  public void SetEnableDrive(boolean value) {
    enableDrive = value;
  }

  public boolean GetEnableDrive() {
    return enableDrive;
  }

  public void SetEnableSteer(boolean value) {
    enableSteer = value;
  }

  public boolean GetEnableSteer() {
    return enableSteer;
  }

  public void SetEnableDriveTrain(boolean value) {
    enableDriveTrain = value;
  }

  public boolean GetEnableDriveTrain() {
    return enableDriveTrain;
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
    speedDilation = Math.abs(value) < speedDilationLimit ? value : speedDilationLimit * sign;
  }

  public double ApplyModifiers(double value) {
    return ApplyModifiers(value, true);
  }

  public double ApplyModifiers(double value, boolean affectSpeed) {
    // disable inversion for now
    // value *= inverseValue;
    var thisSpeedMod = speedMod;

    if (!speedLock && speedDilation < 0)
      // get the inverse of the Y value subtracted from 1 since the axis is reflected along the X axis
      // >= 0 is always full speed, anything less is a fraction of full speed starting at 1 down to 0
      thisSpeedMod = 1 - -speedDilation;

    if (affectSpeed)
      value *= dividerValue * thisSpeedMod;
    
    return value;
  }

  public void ProcessDrive(boolean isAuto) {
    driveModule.ProcessState(isAuto);
  }

  public void ProcessState(boolean isAuto) {
    if (!isAuto) {
      controller.ProcessButtons();
    }

    ProcessDrive(isAuto);

    for (RobotModule module : modules.values()) {
      module.ProcessState(isAuto);
    }
  }

  public void setSpeedMod(double newSpeed) {
    if (newSpeed > 0.0)
      speedMod = newSpeed;
  }

  public void setInverseValue(double newInverse) {
    inverseValue = newInverse;
  }

  public double getInverseValue() {
    return inverseValue;
  }

  public Pose3d GetPosition() {
    return driveModule.GetPosition();
  }

  public void SetTargetActionPose(ActionPose actionPose) {
    SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
  }    

  public void SetTargetActionPose(Group group, int location, int locationIndex, int position, Action action) {
    for (RobotModule module : modules.values()) {
      module.SetTargetActionPose(group, location, locationIndex, position, action);
    }
    driveModule.SetTargetActionPose(group, location, locationIndex, position, action);
  }

  public boolean GetTarget() {
    boolean hasTarget = false;

    if (driveModule.GetTarget() != null) {
      hasTarget = true;
    }

    for (RobotModule module : modules.values()) {
      if (module.GetTarget() != null) {
        hasTarget = true;
      }
    }

    return hasTarget;
  }

    // Abort all active module targets (drive/elevator/actuators) safely
    public void AbandonAllTargets() {
        for (var module : modules.values()) {
            try {
                module.AbandonTarget();
            } catch (Throwable t) {
                // ignore and continue
            }
        }
        
        driveModule.AbandonTarget();

        // also ensure drive open-loop is zeroed if applicable
        try {
            GetDriveModule().ProcessForwardSpeed(0.0);
            GetDriveModule().ProcessLateralSpeed(0.0);
            GetDriveModule().ProcessRotationAngle(0.0);
        } catch (Throwable t) {
            // ignore
        }
    }
}
