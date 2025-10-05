package frc.robot.s2025.Sequences;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.action.Location;
import frc.robot.action.Position;
import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventTarget;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoEvent.EventType;
import frc.robot.modules.ModuleController;

public class SequenceControllerMoveToReef extends AutoSequence {
  public SequenceControllerMoveToReef(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);


    // Target Poses
    var scoreposition1 = new ActionPose(Group.Score, Location.Reef, 0, Position.Lower, Action.Pickup, new Pose3d());
    // var scoreaction1 = new ActionPose(Group.Score, Location.Reef, 0, Position.Lower, Action.Drop, new Pose3d());
    // var pickupposition1 = new ActionPose(Group.Pickup, Location.Coral, 0, Position.Trough, Action.Pickup, new Pose3d());
    // var scoreposition2 = new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Pickup, new Pose3d());
    // var scoreaction2 = new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Drop, new Pose3d());

    AutoEventTarget setScorePosition1 = new AutoEventTarget("Set Score Position 1", true, scoreposition1, AutoEvent.EventType.SetTarget, ac);
    setScorePosition1.SetModuleController(modules);
    AddEvent(setScorePosition1);

    AutoEventTarget awaitScorePosition1 = new AutoEventTarget("Await Score Position 1", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitScorePosition1.SetModuleController(modules);
    AddEvent(awaitScorePosition1);

    // AutoEventTarget setScoreAction1 = new AutoEventTarget("Set Score Action 1", true, scoreaction1, AutoEvent.EventType.SetTarget, ac);
    // setScoreAction1.moduleController = modules;
    // AddEvent(setScoreAction1);

    // AutoEventTarget awaitScoreAction1 = new AutoEventTarget("Await Score Action 1", false, null, AutoEvent.EventType.AwaitTarget, ac);
    // awaitScoreAction1.moduleController = modules;
    // AddEvent(awaitScoreAction1);

    // AutoEventTarget setPickupPosition1 = new AutoEventTarget("Set Pickup Position 1", true, pickupposition1, AutoEvent.EventType.SetTarget, ac);
    // setPickupPosition1.moduleController = modules;
    // AddEvent(setPickupPosition1);

    // AutoEventTarget awaitPickupPosition1 = new AutoEventTarget("Await Pickup Position 1", false, null, AutoEvent.EventType.AwaitTarget, ac);
    // awaitPickupPosition1.moduleController = modules;
    // AddEvent(awaitPickupPosition1);

    // AutoEventTarget setScorePosition2 = new AutoEventTarget("Set Score Position 2", true, scoreposition2, AutoEvent.EventType.SetTarget, ac);
    // setScorePosition2.moduleController = modules;
    // AddEvent(setScorePosition2);

    // AutoEventTarget awaitScorePosition2 = new AutoEventTarget("Await Score Position 2", false, null, AutoEvent.EventType.AwaitTarget, ac);
    // awaitScorePosition2.moduleController = modules;
    // AddEvent(awaitScorePosition2);

    // AutoEventTarget setScoreAction2 = new AutoEventTarget("Set Score Action 2", true, scoreaction2, AutoEvent.EventType.SetTarget, ac);
    // setScoreAction2.moduleController = modules;
    // AddEvent(setScoreAction2);

    // AutoEventTarget awaitScoreAction2 = new AutoEventTarget("Await Score Action 2", false, null, AutoEvent.EventType.AwaitTarget, ac);
    // awaitScoreAction2.moduleController = modules;
    // AddEvent(awaitScoreAction2);
  }
}