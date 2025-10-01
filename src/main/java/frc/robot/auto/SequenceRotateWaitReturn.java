package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.ModuleController;
import frc.robot.SwerveDriveModule;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.action.Location;
import frc.robot.action.Position;

public class SequenceRotateWaitReturn extends AutoSequence {
  public SequenceRotateWaitReturn(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    // Modules
    var drive = (SwerveDriveModule) modules.GetDriveModule();

    // Drive poses (rotation only; zero translation)
    var currentTrans = drive.GetPosition().getTranslation();
    ActionPose rotate90 = new ActionPose(
      Group.Any, Location.Any, 0, Position.Any, Action.Any,
      new Pose3d(currentTrans, new Rotation3d(0, 0, Units.degreesToRadians(90)))
    );
    ActionPose rotate0 = new ActionPose(
      Group.Any, Location.Any, 1, Position.Any, Action.Any,
      new Pose3d(currentTrans, new Rotation3d(0, 0, 0))
    );
    drive.AddActionPose(rotate90);
    drive.AddActionPose(rotate0);

    // Phase 1: Dispatch both targets in parallel
    AutoEventTarget setDrive90 = new AutoEventTarget("Set Drive 90deg", false, rotate90, AutoEvent.EventType.SetTarget, ac);
    setDrive90.targetModule = drive;
    AddEvent(setDrive90);

    // Phase 2: Await completion
    AutoEventTarget awaitDrive90 = new AutoEventTarget("Await Drive 90deg", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive90.targetModule = drive;
    AddEvent(awaitDrive90);

    AutoEventTime waitTime = new AutoEventTime("Wait 10 seconds", false, 10000, AutoEvent.EventType.Void, ac);
    AddEvent(waitTime);

    AutoEventTarget setDrive0 = new AutoEventTarget("Set Drive 0deg", false, rotate0, AutoEvent.EventType.SetTarget, ac);
    setDrive0.targetModule = drive;
    AddEvent(setDrive0);

    // Phase 5: Await completion
    AutoEventTarget awaitDrive0 = new AutoEventTarget("Await Drive 0deg", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive0.targetModule = drive;
    AddEvent(awaitDrive0);
  }
}