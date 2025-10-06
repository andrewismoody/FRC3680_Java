package frc.robot.z2025.Sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoTarget;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SwerveDriveModule;

// SequenceRotateWaitReturn is a simple example auto sequence that uses target events mixed with timed events to move the robot.
public class SequenceRotateWaitReturn extends AutoSequence {
  public SequenceRotateWaitReturn(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    // Modules
    var drive = (SwerveDriveModule) modules.GetDriveModule();

    // Drive poses (rotation only; zero translation)
    ActionPose rotate90 = new ActionPose(
      Group.Any, Location.Any.getValue(), 0, Position.Any.getValue(), Action.Any,
      new AutoTarget(new Rotation2d(Units.degreesToRadians(90)))
    );
    ActionPose rotate0 = new ActionPose(
      Group.Any, Location.Any.getValue(), 1, Position.Any.getValue(), Action.Any,
      new AutoTarget(new Rotation2d(0))
    );
    drive.AddActionPose(rotate90);
    drive.AddActionPose(rotate0);

    // Phase 1: Dispatch both targets in parallel
    AutoEventTarget setDrive90 = new AutoEventTarget("Set Drive 90deg", false, rotate90, AutoEvent.EventType.SetTarget, ac);
    setDrive90.SetTargetModule(drive);
    AddEvent(setDrive90);

    // Phase 2: Await completion
    AutoEventTarget awaitDrive90 = new AutoEventTarget("Await Drive 90deg", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive90.SetTargetModule(drive);
    AddEvent(awaitDrive90);

    AutoEventTime waitTime = new AutoEventTime("Wait 10 seconds", false, 10000, AutoEvent.EventType.Void, ac);
    AddEvent(waitTime);

    AutoEventTarget setDrive0 = new AutoEventTarget("Set Drive 0deg", false, rotate0, AutoEvent.EventType.SetTarget, ac);
    setDrive0.SetTargetModule(drive);
    AddEvent(setDrive0);

    // Phase 5: Await completion
    AutoEventTarget awaitDrive0 = new AutoEventTarget("Await Drive 0deg", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive0.SetTargetModule(drive);
    AddEvent(awaitDrive0);
  }
}