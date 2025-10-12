package frc.robot.z2025.Sequences;

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

public class SequenceControllerStartScoreReload extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public SequenceControllerStartScoreReload(String label, AutoController ac) {
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

    // Target Poses
    var pose_start1 = new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint12_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint11_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint1_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align11_Reef = new ActionPose(Group.Align, Location.Reef.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align12_Reef = new ActionPose(Group.Align, Location.Reef.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var pose_approach11_Reef = new ActionPose(Group.Approach, Location.Reef.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var pose_approach12_Reef = new ActionPose(Group.Approach, Location.Reef.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score11_Lower = new ActionPose(Group.Score, Location.Reef.getValue(), 11, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score12_Lower = new ActionPose(Group.Score, Location.Reef.getValue(), 12, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score11_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score12_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align1_coral = new ActionPose(Group.Align, Location.Coral.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_pickup1_coral = new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);

    // var drive = modules.GetDriveModule();
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    var event_start1 = CreateSyncAwaitEvent("Await Pose Start 1", pose_start1);
    var event_waypoint12_reef1 = CreateSyncAwaitEvent("Await Pose Waypoint 12 Reef 1", pose_waypoint12_Reef);
    var event_waypoint12_reef2 = CreateSyncAwaitEvent("Await Pose Waypoint 12 Reef 2", pose_waypoint12_Reef);
    var event_waypoint12_reef3 = CreateSyncAwaitEvent("Await Pose Waypoint 12 Reef 3", pose_waypoint12_Reef);
    var event_waypoint11_reef1 = CreateSyncAwaitEvent("Await Pose Waypoint 11 Reef 1", pose_waypoint11_Reef);
    var event_waypoint11_reef2 = CreateSyncAwaitEvent("Await Pose Waypoint 11 Reef 2", pose_waypoint11_Reef);
    var event_waypoint11_reef3 = CreateSyncAwaitEvent("Await Pose Waypoint 11 Reef 3", pose_waypoint11_Reef);
    var event_waypoint11_reef4 = CreateSyncAwaitEvent("Await Pose Waypoint 11 Reef 4", pose_waypoint11_Reef);
    var event_waypoint1_reef1 = CreateSyncAwaitEvent("Await Pose Waypoint 1 Reef 1", pose_waypoint1_Reef);
    var event_waypoint1_reef2 = CreateSyncAwaitEvent("Await Pose Waypoint 1 Reef 2", pose_waypoint1_Reef);
    var event_align11_reef1 = CreateSyncAwaitEvent("Await Pose Align 11 Reef 1", pose_align11_Reef);
    var event_align12_reef1 = CreateSyncAwaitEvent("Await Pose Align 12 Reef 1", pose_align12_Reef);
    var event_approach11_reef1 = CreateSyncAwaitEvent("Await Pose Approach 11 Reef 1", pose_approach11_Reef);
    var event_approach12_reef1 = CreateSyncAwaitEvent("Await Pose Approach 12 Reef 1", pose_approach12_Reef);
    var event_align1_coral1 = CreateSyncAwaitEvent("Await Pose Align 1 Coral 1", pose_align1_coral);
    var event_pickup1_coral = CreateSyncAwaitEvent("Await Pose Pickup 1 Coral", pose_pickup1_coral);

    AutoEventTarget event_score11_lower1 = CreateSyncAwaitEvent("Await Pose Score 11 Lower 1", pose_score11_Lower);
    AutoEventTarget event_score12_lower1 = CreateSyncAwaitEvent("Await Pose Score 12 Lower 1", pose_score12_Lower);

    AutoEventTarget event_score11_trough1 = CreateSyncAwaitEvent("Await Pose Score 11 Trough 1", pose_score11_Trough);
    AutoEventTarget event_score12_trough1 = CreateSyncAwaitEvent("Await Pose Score 12 Trough 1", pose_score12_Trough);

    AutoEventTime event_openLatch1 = new AutoEventTime("Open Latch 1", false, 2000, AutoEvent.EventType.Boolean, autoController);
    event_openLatch1.SetBoolEvent(true, slide::ApplyValue);
    AutoEventTime event_openLatch2 = new AutoEventTime("Open Latch 2", false, 2000, AutoEvent.EventType.Boolean, autoController);
    event_openLatch2.SetBoolEvent(true, slide::ApplyValue);

    AutoEventTime event_closeLatchAfter1 = new AutoEventTime("Close Latch (after 2s) 1", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_closeLatchAfter1.SetBoolEvent(true, slide::ApplyInverse);
    AutoEventTime event_closeLatchAfter2 = new AutoEventTime("Close Latch (after 2s) 2", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_closeLatchAfter2.SetBoolEvent(true, slide::ApplyInverse);

    AutoEventTime event_waitForLoading = new AutoEventTime("Wait For Loading", false, 5000, AutoEvent.EventType.None, autoController);

    BeginWith(event_start1)
      .Then(event_waypoint12_reef1)
      .Then(event_waypoint11_reef1)
      .Then(event_align11_reef1)
      .Then(event_approach11_reef1)
      .Then(event_score11_lower1)
      .Then(event_openLatch1)
      .Then(event_closeLatchAfter1)
      .Then(event_score11_trough1)
      .Then(event_waypoint11_reef2)
      .Then(event_waypoint12_reef2)
      .Then(event_waypoint1_reef1)
      .Then(event_align1_coral1)
      .Then(event_pickup1_coral)
      .Then(event_waitForLoading)
      .Then(event_waypoint1_reef2)
      .Then(event_waypoint12_reef3)
      .Then(event_waypoint11_reef3)
      .Then(event_align12_reef1)
      .Then(event_approach12_reef1)
      .Then(event_score12_lower1)
      .Then(event_openLatch2)
      .Then(event_closeLatchAfter2)
      .Then(event_score12_trough1)
      .Then(event_waypoint11_reef4)
    ;
  }

  AutoEventTarget CreateSyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, false, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }

  AutoEventTarget CreateAsyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, true, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }
}