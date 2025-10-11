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
    var pose_waypoint1_Reef = new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score1_Reef = new ActionPose(Group.Align, Location.Reef.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_score_Middle = new ActionPose(Group.Score, Location.Any.getValue(), 1, Position.Middle.getValue(), Action.Pickup, null);
    var pose_score_Lower = new ActionPose(Group.Score, Location.Any.getValue(), 1, Position.Lower.getValue(), Action.Pickup, null);
    var pose_score_Trough = new ActionPose(Group.Score, Location.Any.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);
    var pose_pickup1_coral = new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Trough.getValue(), Action.Pickup, null);

    var drive = modules.GetDriveModule();
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    var event_start1 = CreateSyncAwaitEvent("Await Pose Start 1", pose_start1);
    var event_waypoint12_reef = CreateSyncAwaitEvent("Await Pose Waypoint 12 Reef", pose_waypoint12_Reef);
    var event_waypoint1_reef1 = CreateSyncAwaitEvent("Await Pose Waypoint 1 Reef 1", pose_waypoint1_Reef);
    var event_waypoint1_reef2 = CreateSyncAwaitEvent("Await Pose Waypoint 1 Reef 2", pose_waypoint1_Reef);
    var event_waypoint1_reef3 = CreateSyncAwaitEvent("Await Pose Waypoint 1 Reef 3", pose_waypoint1_Reef);
    var event_score1_reef1 = CreateSyncAwaitEvent("Await Pose Score 1 Reef 1", pose_score1_Reef);
    var event_score1_reef2 = CreateSyncAwaitEvent("Await Pose Score 1 Reef 2", pose_score1_Reef);
    var event_pickup1_coral = CreateSyncAwaitEvent("Await Pose Pickup 1 Coral", pose_pickup1_coral);

    AutoEventTime event_driveLeft1 = new AutoEventTime("Drive Left 1", false, 2000, AutoEvent.EventType.Double, autoController);
    event_driveLeft1.SetDoubleEvent(0.5, drive::ProcessLateralSpeed); // positive = left
    AutoEventTime event_driveLeft2 = new AutoEventTime("Drive Left 2", false, 2000, AutoEvent.EventType.Double, autoController);
    event_driveLeft2.SetDoubleEvent(0.5, drive::ProcessLateralSpeed); // positive = left
    AutoEventTime event_driveLeft3 = new AutoEventTime("Drive Left 3", false, 2000, AutoEvent.EventType.Double, autoController);
    event_driveLeft3.SetDoubleEvent(0.5, drive::ProcessLateralSpeed); // positive = left

    AutoEventTime event_driveRight1 = new AutoEventTime("Drive Right", false, 2000, AutoEvent.EventType.Double, autoController);
    event_driveRight1.SetDoubleEvent(-0.5, drive::ProcessLateralSpeed); // negative = right
    AutoEventTime event_driveRight2 = new AutoEventTime("Drive Right", false, 2000, AutoEvent.EventType.Double, autoController);
    event_driveRight2.SetDoubleEvent(-0.5, drive::ProcessLateralSpeed); // negative = right
    AutoEventTime event_driveRight3 = new AutoEventTime("Drive Right", false, 2000, AutoEvent.EventType.Double, autoController);
    event_driveRight3.SetDoubleEvent(-0.5, drive::ProcessLateralSpeed); // negative = right

    AutoEventTime event_enableFieldOriented1 = new AutoEventTime("Enable Field Oriented 1", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_enableFieldOriented1.SetBoolEvent(true, drive::SetFieldOriented);
    AutoEventTime event_enableFieldOriented2 = new AutoEventTime("Enable Field Oriented 2", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_enableFieldOriented2.SetBoolEvent(true, drive::SetFieldOriented);
    AutoEventTime event_enableFieldOriented3 = new AutoEventTime("Enable Field Oriented 3", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_enableFieldOriented3.SetBoolEvent(true, drive::SetFieldOriented);
    AutoEventTime event_enableFieldOriented4 = new AutoEventTime("Enable Field Oriented 4", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_enableFieldOriented4.SetBoolEvent(true, drive::SetFieldOriented);
    AutoEventTime event_enableFieldOriented5 = new AutoEventTime("Enable Field Oriented 5", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_enableFieldOriented5.SetBoolEvent(true, drive::SetFieldOriented);
    AutoEventTime event_enableFieldOriented6 = new AutoEventTime("Enable Field Oriented 6", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_enableFieldOriented6.SetBoolEvent(true, drive::SetFieldOriented);

    AutoEventTime event_disableFieldOriented1 = new AutoEventTime("Disable Field Oriented 1", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_disableFieldOriented1.SetBoolEvent(false, drive::SetFieldOriented);
    AutoEventTime event_disableFieldOriented2 = new AutoEventTime("Disable Field Oriented 2", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_disableFieldOriented2.SetBoolEvent(false, drive::SetFieldOriented);
    AutoEventTime event_disableFieldOriented3 = new AutoEventTime("Disable Field Oriented 3", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_disableFieldOriented3.SetBoolEvent(false, drive::SetFieldOriented);
    AutoEventTime event_disableFieldOriented4 = new AutoEventTime("Disable Field Oriented 4", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_disableFieldOriented4.SetBoolEvent(false, drive::SetFieldOriented);
    AutoEventTime event_disableFieldOriented5 = new AutoEventTime("Disable Field Oriented 5", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_disableFieldOriented5.SetBoolEvent(false, drive::SetFieldOriented);
    AutoEventTime event_disableFieldOriented6 = new AutoEventTime("Disable Field Oriented 6", false, 60, AutoEvent.EventType.Boolean, autoController);
    event_disableFieldOriented6.SetBoolEvent(false, drive::SetFieldOriented);

    AutoEventTarget event_score_lower1 = CreateSyncAwaitEvent("Elevator to Lower 1", pose_score_Lower);
    AutoEventTarget event_score_middle1 = CreateSyncAwaitEvent("Elevator to Middle 1", pose_score_Middle);

    AutoEventTarget event_score_trough1 = CreateSyncAwaitEvent("Elevator to Trough 1", pose_score_Trough);
    AutoEventTarget event_score_trough2 = CreateSyncAwaitEvent("Elevator to Trough 2", pose_score_Trough);

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
      .Then(event_waypoint12_reef)
      .Then(event_waypoint1_reef1)
      .Then(event_score1_reef1)
      .Then(event_disableFieldOriented1)
      .Then(event_driveLeft1)
      .Then(event_enableFieldOriented1)
      .Then(event_score_lower1)
      .Then(event_openLatch1)
      .Then(event_closeLatchAfter1)
      .Then(event_score_trough1)
      .Then(event_disableFieldOriented2)
      .Then(event_driveRight1)
      .Then(event_enableFieldOriented2)
      .Then(event_waypoint1_reef2)
      .Then(event_pickup1_coral)
      .Then(event_disableFieldOriented3)
      .Then(event_driveLeft2)
      .Then(event_enableFieldOriented3)
      .Then(event_waitForLoading)
      .Then(event_disableFieldOriented4)
      .Then(event_driveRight2)
      .Then(event_enableFieldOriented4)
      .Then(event_waypoint1_reef3)
      .Then(event_score1_reef2)
      .Then(event_disableFieldOriented5)
      .Then(event_driveLeft3)
      .Then(event_enableFieldOriented5)
      .Then(event_score_middle1)
      .Then(event_openLatch2)
      .Then(event_closeLatchAfter2)
      .Then(event_score_trough2)
      .Then(event_disableFieldOriented6)
      .Then(event_driveRight3)
      .Then(event_enableFieldOriented6)
    ;
  }

  AutoEventTarget CreateSyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, false, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }

  AutoEventTarget CreateAsyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, true, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }
}