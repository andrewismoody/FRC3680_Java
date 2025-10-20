package frc.robot.z2024.Sequences;

import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.z2024.action.Location;
import frc.robot.z2024.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.misc.Utility;
import frc.robot.modules.SingleMotorModule;

public class ScoreSpeaker extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public ScoreSpeaker(String label, AutoController ac) {
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

    var modules = autoController.GetModuleController();
    var driverLocation = Utility.getDriverLocation();

    // Target Poses
    var pose_start1 = new ActionPose(Group.Start, Location.Start.getValue(), driverLocation, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint4_Stage = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint3_Stage = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 3, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint2_Stage = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 2, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint1_Stage = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align3_Reef = new ActionPose(Group.AlignLeft, Location.Stage.getValue(), 3, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align4_Reef = new ActionPose(Group.AlignLeft, Location.Stage.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_approach3_Reef = new ActionPose(Group.ApproachLeft, Location.Stage.getValue(), 3, Position.Trough.getValue(), Action.Pickup, null);
    var pose_approach4_Reef = new ActionPose(Group.ApproachLeft, Location.Stage.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score3_Lower = new ActionPose(Group.Score, Location.Stage.getValue(), 3, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score4_Lower = new ActionPose(Group.Score, Location.Stage.getValue(), 4, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score3_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 3, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score4_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align1_Speaker = new ActionPose(Group.Align, Location.Speaker.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_pickup1_Speaker = new ActionPose(Group.Pickup, Location.Speaker.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);

    // var drive = modules.GetDriveModule();
    var shoot = (SingleMotorModule) modules.GetModule("shoot");

    var event_start1 = CreateSyncAwaitEvent("Await Pose Start 1", pose_start1);
    var event_waypoint4_stage = CreateSyncAwaitEvent("Await Pose Waypoint 4 Stage", pose_waypoint4_Stage);
    var event_waypoint3_stage = CreateSyncAwaitEvent("Await Pose Waypoint 3 Stage", pose_waypoint3_Stage);
    var event_waypoint2_stage = CreateSyncAwaitEvent("Await Pose Waypoint 2 Stage", pose_waypoint2_Stage);
    var event_waypoint1_stage = CreateSyncAwaitEvent("Await Pose Waypoint 1 Stage", pose_waypoint1_Stage);
    var event_align3_reef = CreateSyncAwaitEvent("Await Pose Align 3 Stage", pose_align3_Reef);
    var event_align4_reef = CreateSyncAwaitEvent("Await Pose Align 4 Stage", pose_align4_Reef);
    var event_approach3_reef = CreateSyncAwaitEvent("Await Pose Approach 3 Stage", pose_approach3_Reef);
    var event_approach4_reef = CreateSyncAwaitEvent("Await Pose Approach 4 Stage", pose_approach4_Reef);
    var event_align1_speaker = CreateSyncAwaitEvent("Await Pose Align 1 Speaker", pose_align1_Speaker);
    var event_pickup1_speaker = CreateSyncAwaitEvent("Await Pose Pickup 1 Speaker", pose_pickup1_Speaker);

    AutoEventTarget event_score3_lower = CreateSyncAwaitEvent("Await Pose Score 3 Lower", pose_score3_Lower);
    AutoEventTarget event_score4_lower = CreateSyncAwaitEvent("Await Pose Score 4 Lower", pose_score4_Lower);

    AutoEventTarget event_score3_trough = CreateSyncAwaitEvent("Await Pose Score 3 Trough", pose_score3_Trough);
    AutoEventTarget event_score4_trough = CreateSyncAwaitEvent("Await Pose Score 4 Trough", pose_score4_Trough);

    AutoEventTime event_openLatch = new AutoEventTime("Open Latch", false, 2000, AutoEvent.EventType.Boolean, autoController);
    event_openLatch.SetBoolEvent(true, shoot::ApplyValue);

    AutoEventTime event_closeLatchAfter = new AutoEventTime("Close Latch (after 2s)", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_closeLatchAfter.SetBoolEvent(true, shoot::ApplyInverse);

    AutoEventTime event_waitForLoading = new AutoEventTime("Wait For Loading", false, 5000, AutoEvent.EventType.None, autoController);

    BeginWith(event_start1);

    switch (driverLocation) {
      case 1:
        Then(event_waypoint1_stage);
      case 2:
        Then(event_waypoint2_stage);
      case 3:
        Then(event_waypoint3_stage);
        break;
    }

    Then(event_waypoint4_stage)
      .Then(event_align3_reef)
      .Then(event_approach3_reef)
      .Then(event_score3_lower)
      .Then(event_openLatch)
      .Then(event_closeLatchAfter)
      .Then(event_score3_trough)
      .Then(event_waypoint2_stage)
      .Then(event_waypoint1_stage)
      .Then(event_align1_speaker)
      .Then(event_pickup1_speaker)
      .Then(event_waitForLoading)
      .Then(event_waypoint2_stage)
      .Then(event_align4_reef)
      .Then(event_approach4_reef)
      .Then(event_score4_lower)
      .Then(event_openLatch)
      .Then(event_closeLatchAfter)
      .Then(event_score4_trough)
      .Then(event_waypoint2_stage)
    ;
  }

  AutoEventTarget CreateSyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, false, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }

  AutoEventTarget CreateAsyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, true, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }
}