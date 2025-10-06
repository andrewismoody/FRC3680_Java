package frc.robot.z2025.Sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.action.Location;
import frc.robot.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoTarget;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.SwerveDriveModule;

// SequenceRotateScoreReturn is an example auto sequence that uses target events mixed with timed events to control multiple modules.
public class SequenceRotateScoreReturn extends AutoSequence {
  public SequenceRotateScoreReturn(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    // Modules
    var drive = (SwerveDriveModule) modules.GetDriveModule();
    var elevator = (SingleMotorModule) modules.GetModule("elevator");
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    // Drive poses (rotation only; use current translation)
    var currentTrans = drive.GetPosition().getTranslation();
    ActionPose rotate90 = new ActionPose(
      Group.Any, Location.Any, 0, Position.Any, Action.Any,
      new AutoTarget(currentTrans, new Rotation2d(Units.degreesToRadians(90)))
    );
    ActionPose rotate0 = new ActionPose(
      Group.Any, Location.Any, 1, Position.Any, Action.Any,
      new AutoTarget(currentTrans, new Rotation2d(Units.degreesToRadians(0)))
    );
    drive.AddActionPose(rotate90);
    drive.AddActionPose(rotate0);

    // Elevator poses (assumes these were added in robotInit; Middle=L2, Trough=zero)
    var elevL2 = elevator.GetActionPose(Group.Score, Location.Any, -1, Position.Middle, Action.Any);
    var elevZero = elevator.GetActionPose(Group.Score, Location.Any, -1, Position.Trough, Action.Any);

    // Phase 1: Dispatch both targets in parallel
    AutoEventTarget setDrive90 = new AutoEventTarget("Set Drive 90deg", true, rotate90, AutoEvent.EventType.SetTarget, ac);
    setDrive90.SetTargetModule(drive);
    AddEvent(setDrive90);

    AutoEventTarget setElevL2 = new AutoEventTarget("Set Elevator L2", true, elevL2, AutoEvent.EventType.SetTarget, ac);
    setElevL2.SetTargetModule(elevator);
    AddEvent(setElevL2);

    // Phase 2: Await both completions in parallel
    AutoEventTarget awaitDrive90 = new AutoEventTarget("Await Drive 90deg", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive90.SetTargetModule(drive);
    AddEvent(awaitDrive90);

    AutoEventTarget awaitElevL2 = new AutoEventTarget("Await Elevator L2", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitElevL2.SetTargetModule(elevator);
    AddEvent(awaitElevL2);

    // Phase 3: Open slide latch for 2 seconds
    AutoEventTime openLatch = new AutoEventTime("Open Latch", false, 0, AutoEvent.EventType.Boolean, ac);
    openLatch.SetBoolEvent(true, slide::ApplyValue);
    AddEvent(openLatch);

    AutoEventTime closeLatchAfter = new AutoEventTime("Close Latch (after 2s)", false, 2000, AutoEvent.EventType.Boolean, ac);
    closeLatchAfter.SetBoolEvent(true, slide::ApplyInverse); // true => reverse/close
    AddEvent(closeLatchAfter);

    // Phase 4: Simultaneously dispatch elevator to zero and rotate back to 0deg
    AutoEventTarget setElevZero = new AutoEventTarget("Set Elevator Zero", false, elevZero, AutoEvent.EventType.SetTarget, ac);
    setElevZero.SetTargetModule(elevator);
    AddEvent(setElevZero);

    AutoEventTarget awaitElevZero = new AutoEventTarget("Await Elevator Zero", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitElevZero.SetTargetModule(elevator);
    AddEvent(awaitElevZero);

    AutoEventTarget setDrive0 = new AutoEventTarget("Set Drive 0deg", false, rotate0, AutoEvent.EventType.SetTarget, ac);
    setDrive0.SetTargetModule(drive);
    AddEvent(setDrive0);

    // Phase 5: Await both completions in parallel
    AutoEventTarget awaitDrive0 = new AutoEventTarget("Await Drive 0deg", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive0.SetTargetModule(drive);
    AddEvent(awaitDrive0);
  }
}