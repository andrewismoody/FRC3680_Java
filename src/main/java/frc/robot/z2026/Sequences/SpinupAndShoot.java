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

public class SpinupAndShoot extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public SpinupAndShoot(String label, AutoController ac) {
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

    var pose_spinup = new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Feed, null);
    var pose_shoot = new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Release, null);
    var pose_stop = new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Any.getValue(), Action.None, null);

    var event_spinup = CreateSyncAwaitEvent("Await Pose Shooter Spinup", pose_spinup);
    var event_shoot = CreateSyncAwaitEvent("Await Pose Feeder Shoot", pose_shoot);
    var event_waitForRelease = new AutoEventTime("Wait For Release", false, 2000, AutoEvent.EventType.None, autoController);
    var event_stop = CreateSyncAwaitEvent("Await Pose Shooter/Feeder Stop", pose_stop);

    BeginWith(event_spinup)
      .Then(event_shoot)
      .Then(event_waitForRelease)
      .Then(event_stop)
    ;
  }

  AutoEventTarget CreateSyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, false, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }

  AutoEventTarget CreateAsyncAwaitEvent(String name, ActionPose pose) {
    return new AutoEventTarget(name, true, pose, AutoEvent.EventType.AwaitTarget, autoController);
  }
}