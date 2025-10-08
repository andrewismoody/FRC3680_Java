package frc.robot.z2025.Sequences;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Hashtable;
import java.util.List;

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
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;

public class SequenceControllerStartScoreReload extends AutoSequence {
  public SequenceControllerStartScoreReload(String label, AutoController ac) {
    super(label, ac);
    var modules = ac.GetModuleController();

    // Target Poses
    var start1 = new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    //var waypoint11_240 = new ActionPose(Group.Score, Location.Waypoint.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var waypoint12_Reef = new ActionPose(Group.Score, Location.Waypoint.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var waypoint1_Reef = new ActionPose(Group.Score, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var score1_Reef = new ActionPose(Group.Score, Location.Reef.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    // var waypoint1_126 = new ActionPose(Group.Pickup, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    // var loading1_126 = new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);

    var drive = modules.GetDriveModule();
    var elevator = (SingleMotorModule) modules.GetModule("elevator");
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    AutoEventTime driveLeft = new AutoEventTime("Drive Left", false, 2000, AutoEvent.EventType.Double, ac);
    driveLeft.SetDoubleEvent(-0.5, drive::ProcessLateralSpeed); // negative = left

    // TODO: this didn't seem to fire
    AutoEventTime enableFieldOriented = new AutoEventTime("Enable Field Oriented", false, 1, AutoEvent.EventType.Boolean, ac);
    enableFieldOriented.SetBoolEvent(true, drive::SetFieldOriented);

    AutoEventTime disableFieldOriented = new AutoEventTime("Disable Field Oriented", false, 1, AutoEvent.EventType.Boolean, ac);
    disableFieldOriented.SetBoolEvent(false, drive::SetFieldOriented);

    AutoEventTarget ElevatorToLower = new AutoEventTarget("Elevator to Lower", false, elevator.GetActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any), AutoEvent.EventType.AwaitTarget, ac);
    AutoEventTarget ElevatorToTrough = new AutoEventTarget("Elevator to Trough", false, elevator.GetActionPose(Group.Score, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any), AutoEvent.EventType.AwaitTarget, ac);

    AutoEventTime openLatch = new AutoEventTime("Open Latch", false, 2000, AutoEvent.EventType.Boolean, ac);
    openLatch.SetBoolEvent(true, slide::ApplyValue);

    AutoEventTime closeLatchAfter = new AutoEventTime("Close Latch (after 2s)", false, 1, AutoEvent.EventType.Boolean, ac);
    closeLatchAfter.SetBoolEvent(true, slide::ApplyInverse); // true => reverse/close

    var Poses = new Hashtable<String, ActionPose>();
    Poses.put("1 Start 1", start1);
    // Poses.put("1 Waypoint 11 240", waypoint11_240);
    Poses.put("2 Waypoint 12 Reef", waypoint12_Reef);
    Poses.put("3 Waypoint 1 Reef", waypoint1_Reef);
    Poses.put("4 Score 1 Reef", score1_Reef);
    // Poses.put("5 Waypoint 1 126", waypoint1_126);
    // Poses.put("6 Loading 1 126", loading1_126);

    List<String> poseKeys = new ArrayList<String>(Poses.keySet());
    Collections.sort(poseKeys);

    for (var key : poseKeys) {
      AutoEventTarget awaitPose = new AutoEventTarget("Await Pose " + key, false, Poses.get(key), AutoEvent.EventType.AwaitTarget, ac);
      AddEvent(awaitPose);
    }

    AddEvent(disableFieldOriented);
    AddEvent(driveLeft);
    AddEvent(ElevatorToLower);
    AddEvent(openLatch);
    AddEvent(closeLatchAfter);
    AddEvent(ElevatorToTrough);
    AddEvent(enableFieldOriented);
  }
}