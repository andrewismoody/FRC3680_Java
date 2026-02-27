package frc.robot.z2026.Sequences;

import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.z2026.action.Location;
import frc.robot.z2026.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.misc.Utility;

public class Score4andReload extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public Score4andReload(String label, AutoController ac) {
    super(label, ac);
    this.autoController = ac;
    // constructor intentionally does not access modules or poses
  }

  // Call this after modules and poses have been registered
  @Override
  public void Initialize() {
    super.Initialize();

    if (initialized) {
      System.out.printf("Sequence %s already initialized; skipping duplicate init\n", GetLabel());
      return;
    }
    initialized = true;

    var driverLocation = Utility.getDriverLocation();

    // Target Poses
    var pose_start1 = new ActionPose(Group.Start, Location.Start.getValue(), driverLocation, Position.Ground.getValue(), Action.None, null);
    var pose_waypoint1 = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Ground.getValue(), Action.None, null);
    var pose_waypoint2 = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 2, Position.Ground.getValue(), Action.None, null);
    var pose_waypoint3 = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 3, Position.Ground.getValue(), Action.None, null);
    var pose_waypoint4 = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 4, Position.Ground.getValue(), Action.None, null);
    var pose_align4_Hub = new ActionPose(Group.Align, Location.Hub.getValue(), 4, Position.Ground.getValue(), Action.None, null);
    var pose_approach4_Hub = new ActionPose(Group.Approach, Location.Hub.getValue(), 4, Position.Ground.getValue(), Action.Pickup, null);
    var pose_score4_feed = new ActionPose(Group.Score, Location.Hub.getValue(), 4, Position.Ground.getValue(), Action.Feed, null);
    var pose_score4_release = new ActionPose(Group.Score, Location.Hub.getValue(), 4, Position.Ground.getValue(), Action.Release, null);
    var pose_align1_Outpost = new ActionPose(Group.Align, Location.Outpost.getValue(), 1, Position.Ground.getValue(), Action.None, null);
    var pose_pickup1_Outpost = new ActionPose(Group.Pickup, Location.Outpost.getValue(), 1, Position.Ground.getValue(), Action.None, null);

    var event_start1 = CreateSyncAwaitEvent("Await Pose Start 1", pose_start1);
    var event_waypoint4 = CreateSyncAwaitEvent("Await Pose Waypoint 4 Hub", pose_waypoint4);
    var event_waypoint3 = CreateSyncAwaitEvent("Await Pose Waypoint 3 Hub", pose_waypoint3);
    var event_waypoint2 = CreateSyncAwaitEvent("Await Pose Waypoint 2 Hub", pose_waypoint2);
    var event_waypoint1 = CreateSyncAwaitEvent("Await Pose Waypoint 1 Hub", pose_waypoint1);
    var event_approach4_Hub = CreateSyncAwaitEvent("Await Pose Approach 4 Hub", pose_approach4_Hub);
    var event_align4_Hub = CreateSyncAwaitEvent("Await Pose Align 4 Hub", pose_align4_Hub);
    var event_score4_feed = CreateSyncAwaitEvent("Await Pose Score 4 Feed Hub", pose_score4_feed);
    var event_score4_release = CreateSyncAwaitEvent("Await Pose Score 4 Release Hub", pose_score4_release);
    var event_align1_Outpost = CreateSyncAwaitEvent("Await Pose Align 1 Outpost", pose_align1_Outpost);
    var event_pickup1_Outpost = CreateSyncAwaitEvent("Await Pose Pickup 1 Outpost", pose_pickup1_Outpost);

    AutoEventTime event_waitForLoading = new AutoEventTime("Wait For Loading", false, 5000, AutoEvent.EventType.None, autoController);

    BeginWith(event_start1)
      .Then(event_waypoint4)
      .Then(event_waypoint3)
      .Then(event_approach4_Hub)
      .Then(event_align4_Hub)
      .Then(event_score4_feed)
      .Then(event_score4_release)
      .Then(event_waypoint3)
      .Then(event_waypoint2)
      .Then(event_waypoint1)
      .Then(event_align1_Outpost)
      .Then(event_pickup1_Outpost)
      .Then(event_waitForLoading);
  }

  AutoEventTarget CreateSyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, false, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }

  AutoEventTarget CreateAsyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, true, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }
}