package frc.robot.z2025.Sequences;

import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoSequence;

public class SequenceControllerMoveToReef extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public SequenceControllerMoveToReef(String label, AutoController ac) {
    super(label, ac);
    autoController = ac;
  }

  @Override
  public void Initialize() {
    super.Initialize();

    if (initialized) {
      System.out.printf("Sequence %s already initialized; skipping duplicate init\n", GetLabel());
      return;
    }
    initialized = true;

    // Target Poses
    var scoreposition1 = new ActionPose(Group.Score, Location.Reef.getValue(), 0, Position.Lower.getValue(), Action.Pickup, null);
    // var scoreaction1 = new ActionPose(Group.Score, Location.Reef, 0, Position.Lower, Action.Drop, new Pose3d());
    // var pickupposition1 = new ActionPose(Group.Pickup, Location.Coral, 0, Position.Trough, Action.Pickup, new Pose3d());
    // var scoreposition2 = new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Pickup, new Pose3d());
    // var scoreaction2 = new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Drop, new Pose3d());

    AutoEventTarget awaitScorePosition1 = new AutoEventTarget("Await Score Position 1", false, scoreposition1, AutoEvent.EventType.AwaitTarget, autoController);
    AddEvent(awaitScorePosition1);

    // AutoEventTarget awaitScoreAction1 = new AutoEventTarget("Await Score Action 1", false, scoreaction1, AutoEvent.EventType.AwaitTarget, ac);
    // awaitScoreAction1.moduleController = modules;
    // AddEvent(awaitScoreAction1);

    // AutoEventTarget awaitPickupPosition1 = new AutoEventTarget("Await Pickup Position 1", false, pickupposition1, AutoEvent.EventType.AwaitTarget, ac);
    // awaitPickupPosition1.moduleController = modules;
    // AddEvent(awaitPickupPosition1);

    // AutoEventTarget awaitScorePosition2 = new AutoEventTarget("Await Score Position 2", false, scoreposition2, AutoEvent.EventType.AwaitTarget, ac);
    // awaitScorePosition2.moduleController = modules;
    // AddEvent(awaitScorePosition2);

    // AutoEventTarget awaitScoreAction2 = new AutoEventTarget("Await Score Action 2", false, scoreaction2, AutoEvent.EventType.AwaitTarget, ac);
    // awaitScoreAction2.moduleController = modules;
    // AddEvent(awaitScoreAction2);
  }
}