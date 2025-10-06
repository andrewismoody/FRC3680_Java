package frc.robot.z2025.Sequences;

import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.action.Location;
import frc.robot.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoSequence;
import frc.robot.modules.ModuleController;

public class SequenceControllerScoreReloadScore extends AutoSequence {
  public SequenceControllerScoreReloadScore(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);


    // Target Poses
    var scoreposition1 = new ActionPose(Group.Score, Location.Reef, 0, Position.Lower, Action.Pickup, null);
    var scoreaction1 = new ActionPose(Group.Score, Location.Reef, 0, Position.Lower, Action.Drop, null);
    var pickupposition1 = new ActionPose(Group.Pickup, Location.Coral, 0, Position.Trough, Action.Pickup, null);
    var scoreposition2 = new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Pickup, null);
    var scoreaction2 = new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Drop, null);

    AutoEventTarget awaitScorePosition1 = new AutoEventTarget("Await Score Position 1", false, scoreposition1, AutoEvent.EventType.AwaitTarget, ac);
    AddEvent(awaitScorePosition1);

    AutoEventTarget awaitScoreAction1 = new AutoEventTarget("Await Score Action 1", false, scoreaction1, AutoEvent.EventType.AwaitTarget, ac);
    AddEvent(awaitScoreAction1);

    AutoEventTarget awaitPickupPosition1 = new AutoEventTarget("Await Pickup Position 1", false, pickupposition1, AutoEvent.EventType.AwaitTarget, ac);
    AddEvent(awaitPickupPosition1);

    AutoEventTarget awaitScorePosition2 = new AutoEventTarget("Await Score Position 2", false, scoreposition2, AutoEvent.EventType.AwaitTarget, ac);
    AddEvent(awaitScorePosition2);

    AutoEventTarget awaitScoreAction2 = new AutoEventTarget("Await Score Action 2", false, scoreaction2, AutoEvent.EventType.AwaitTarget, ac);
    AddEvent(awaitScoreAction2);
  }
}