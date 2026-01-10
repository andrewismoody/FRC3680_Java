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
import frc.robot.misc.Utility;
import frc.robot.modules.SingleActuatorModule;

public class Score8and4 extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public Score8and4(String label, AutoController ac) {
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
    var pose_start = new ActionPose(Group.Start, Location.Barge.getValue(), driverLocation, Position.Trough.getValue(), Action.Pickup, null);
    // var pose_waypoint1_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint3_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 3, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint4_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint5_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 5, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint6_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 6, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint7_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 7, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint8_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 8, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint9_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 9, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint10_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 10, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint11_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 11, Position.Trough.getValue(), Action.Pickup, null);
    var pose_waypoint12_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 12, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align4_Reef = new ActionPose(Group.AlignRight, Location.Reef.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align8_Reef = new ActionPose(Group.AlignRight, Location.Reef.getValue(), 8, Position.Trough.getValue(), Action.Pickup, null);
    var pose_approach4_Reef = new ActionPose(Group.ApproachRight, Location.Reef.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_approach8_Reef = new ActionPose(Group.ApproachRight, Location.Reef.getValue(), 8, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score4_Lower = new ActionPose(Group.Score, Location.Reef.getValue(), 4, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score8_Lower = new ActionPose(Group.Score, Location.Reef.getValue(), 8, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score4_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 4, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score8_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 8, Position.Trough.getValue(), Action.Pickup, null);
    var pose_align2_coral = new ActionPose(Group.Align, Location.Coral.getValue(), 2, Position.Trough.getValue(), Action.Pickup, null);
    var pose_pickup2_coral = new ActionPose(Group.Pickup, Location.Coral.getValue(), 2, Position.Trough.getValue(), Action.Pickup, null);

    // var drive = modules.GetDriveModule();
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    var event_start = CreateSyncAwaitEvent("Await Pose Start", pose_start);
    var event_waypoint12_reef = CreateSyncAwaitEvent("Await Pose Waypoint 12 Reef", pose_waypoint12_Reef);
    var event_waypoint11_reef = CreateSyncAwaitEvent("Await Pose Waypoint 11 Reef", pose_waypoint11_Reef);
    var event_waypoint10_reef = CreateSyncAwaitEvent("Await Pose Waypoint 10 Reef", pose_waypoint10_Reef);
    var event_waypoint9_reef = CreateSyncAwaitEvent("Await Pose Waypoint 9 Reef", pose_waypoint9_Reef);
    var event_waypoint8_reef = CreateSyncAwaitEvent("Await Pose Waypoint 8 Reef", pose_waypoint8_Reef);
    var event_waypoint7_reef = CreateSyncAwaitEvent("Await Pose Waypoint 7 Reef", pose_waypoint7_Reef);
    var event_waypoint6_reef = CreateSyncAwaitEvent("Await Pose Waypoint 6 Reef", pose_waypoint6_Reef);
    var event_waypoint5_reef = CreateSyncAwaitEvent("Await Pose Waypoint 5 Reef", pose_waypoint5_Reef);
    var event_waypoint4_reef = CreateSyncAwaitEvent("Await Pose Waypoint 4 Reef", pose_waypoint4_Reef);
    var event_waypoint3_reef = CreateSyncAwaitEvent("Await Pose Waypoint 3 Reef", pose_waypoint3_Reef);
    // var event_waypoint1_reef = CreateSyncAwaitEvent("Await Pose Waypoint 1 Reef", pose_waypoint1_Reef);
    var event_align8_reef = CreateSyncAwaitEvent("Await Pose Align 8 Reef", pose_align8_Reef);
    var event_align4_reef = CreateSyncAwaitEvent("Await Pose Align 4 Reef", pose_align4_Reef);
    var event_approach8_reef = CreateSyncAwaitEvent("Await Pose Approach 8 Reef", pose_approach8_Reef);
    var event_approach4_reef = CreateSyncAwaitEvent("Await Pose Approach 4 Reef", pose_approach4_Reef);
    var event_align2_coral = CreateSyncAwaitEvent("Await Pose Align 2 Coral", pose_align2_coral);
    var event_pickup2_coral = CreateSyncAwaitEvent("Await Pose Pickup 2 Coral", pose_pickup2_coral);

    AutoEventTarget event_score8_lower = CreateSyncAwaitEvent("Await Pose Score 8 Lower", pose_score8_Lower);
    AutoEventTarget event_score4_lower = CreateSyncAwaitEvent("Await Pose Score 4 Lower", pose_score4_Lower);

    AutoEventTarget event_score8_trough = CreateSyncAwaitEvent("Await Pose Score 8 Trough", pose_score8_Trough);
    AutoEventTarget event_score4_trough = CreateSyncAwaitEvent("Await Pose Score 4 Trough", pose_score4_Trough);

    AutoEventTime event_openLatch = new AutoEventTime("Open Latch", false, 2000, AutoEvent.EventType.Boolean, autoController);
    event_openLatch.SetBoolEvent(true, slide::ApplyValue);

    AutoEventTime event_closeLatchAfter = new AutoEventTime("Close Latch (after 2s)", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_closeLatchAfter.SetBoolEvent(true, slide::ApplyInverse);

    AutoEventTime event_waitForLoading = new AutoEventTime("Wait For Loading", false, 5000, AutoEvent.EventType.None, autoController);

    BeginWith(event_start);
    
    switch (driverLocation) {
      case 1:
        Then(event_waypoint12_reef);
      case 2:
        Then(event_waypoint11_reef);
        break;
      case 3:
        break;
    }

    Then(event_waypoint10_reef)
      .Then(event_waypoint9_reef)
      .Then(event_align8_reef)
      .Then(event_approach8_reef)
      .Then(event_score8_lower)
      .Then(event_openLatch)
      .Then(event_closeLatchAfter)
      .Then(event_score8_trough)
      .Then(event_waypoint8_reef)
      .Then(event_waypoint7_reef)
      .Then(event_waypoint6_reef)
      .Then(event_waypoint5_reef)
      .Then(event_align2_coral)
      .Then(event_pickup2_coral)
      .Then(event_waitForLoading)
      .Then(event_waypoint5_reef)
      .Then(event_waypoint4_reef)
      .Then(event_align4_reef)
      .Then(event_approach4_reef)
      .Then(event_score4_lower)
      .Then(event_openLatch)
      .Then(event_closeLatchAfter)
      .Then(event_score4_trough)
      .Then(event_waypoint3_reef)
    ;
  }

  AutoEventTarget CreateSyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, false, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }

  AutoEventTarget CreateAsyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, true, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }
}