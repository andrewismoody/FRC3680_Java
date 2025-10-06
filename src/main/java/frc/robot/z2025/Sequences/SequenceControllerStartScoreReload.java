package frc.robot.z2025.Sequences;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Hashtable;
import java.util.List;

import frc.robot.modules.ModuleController;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoSequence;

public class SequenceControllerStartScoreReload extends AutoSequence {
  public SequenceControllerStartScoreReload(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    // Target Poses
    // var start1 = new ActionPose(Group.Start, Location.Barge, 1, Position.Lower, Action.Pickup, null);
    var waypoint11_240 = new ActionPose(Group.Score, Location.Waypoint.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var waypoint12_180 = new ActionPose(Group.Score, Location.Waypoint.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var waypoint1_300 = new ActionPose(Group.Score, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var score1_300 = new ActionPose(Group.Score, Location.Reef.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var waypoint1_126 = new ActionPose(Group.Pickup, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var loading1_126 = new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);

    var Poses = new Hashtable<String, ActionPose>();
    // Poses.put("1 Start 1", start1);
    Poses.put("1 Waypoint 11 240", waypoint11_240);
    Poses.put("2 Waypoint 12 180", waypoint12_180);
    Poses.put("3 Waypoint 1 300", waypoint1_300);
    Poses.put("4 Score 1 300", score1_300);
    Poses.put("5 Waypoint 1 126", waypoint1_126);
    Poses.put("6 Loading 1 126", loading1_126);

    List<String> poseKeys = new ArrayList<String>(Poses.keySet());
    Collections.sort(poseKeys);

    for (var key : poseKeys) {
      AutoEventTarget awaitPose = new AutoEventTarget("Await Pose " + key, false, Poses.get(key), AutoEvent.EventType.AwaitTarget, ac);
      AddEvent(awaitPose);
    }
  }
}