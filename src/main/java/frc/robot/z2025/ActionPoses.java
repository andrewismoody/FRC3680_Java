package frc.robot.z2025;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.misc.Utility;
import frc.robot.modules.ModuleState;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.SwerveDriveModule;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;

public class ActionPoses {
    public static void Initialize(SwerveDriveModule swerveDriveModule, SingleMotorModule elevator, SingleActuatorModule slide) {
        // TODO: use lookat for reef center and driver station as source
        // start, 240 (we don't know where we are yet, so rotate a specific angle to face a tag)
        swerveDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Utility.getLookat(Constants.getStartPose().getTranslation().toTranslation2d(), Constants.getFieldPosition(Group.Any, Location.Interest, 1).toTranslation2d()).minus(Rotation2d.fromDegrees(90)))));
        // waypoint 11, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), 2, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, 2), Constants.getFieldPosition(Group.Any, Location.Interest, 1))));
        // waypoint 12, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), 12, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, 12), Constants.getFieldPosition(Group.Any, Location.Interest, 1))));
        // waypoint 1, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, 1), Constants.getFieldPosition(Group.Any, Location.Interest, 1))));

        // align scoring 3, match tag 20 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Align, Location.Reef.getValue(), 3, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Reef, 3), Constants.getKnownRotation(Group.Any, Location.Tag, 18).plus(Rotation2d.fromDegrees(180)))));
        // align scoring 4, match tag 20 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Align, Location.Reef.getValue(), 4, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Reef, 4), Constants.getKnownRotation(Group.Any, Location.Tag, 18).plus(Rotation2d.fromDegrees(180)))));

        // approach scoring 3, match tag 20 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Approach, Location.Reef.getValue(), 3, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Approach, Location.Reef, 3), Constants.getKnownRotation(Group.Any, Location.Tag, 18).plus(Rotation2d.fromDegrees(180)))));
        // approach scoring 4, match tag 20 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Approach, Location.Reef.getValue(), 4, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Approach, Location.Reef, 4), Constants.getKnownRotation(Group.Any, Location.Tag, 18).plus(Rotation2d.fromDegrees(180)))));

        // scoring 3, match tag 20 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef.getValue(), 3, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Reef, 3), Constants.getKnownRotation(Group.Any, Location.Tag, 18).plus(Rotation2d.fromDegrees(180)))));
        // scoring 4, match tag 20 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef.getValue(), 4, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Reef, 4), Constants.getKnownRotation(Group.Any, Location.Tag, 18).plus(Rotation2d.fromDegrees(180)))));

        // TODO 1: seek tag breaks this - will have to hard code a rotation like start to switch lookat targets - or fix seek tag
        // waypoint 1, lookat reef -- should this be hard-coded to a rotation or will LookAt work as long as we don't "clear" our position?
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, 1), Constants.getFieldPosition(Group.Any, Location.Interest, 1))));

        // align coral 1, match tag 13 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Align, Location.Coral.getValue(), 1, Position.Any.getValue(), Action.Any,
        new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Coral, 1), Constants.getKnownRotation(Group.Any, Location.Tag, 13))));

        // coral 1, match tag 13 rotation
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Coral, 1), Constants.getKnownRotation(Group.Any, Location.Tag, 13))));

        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.36)));
        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.55)));
        elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Drop, new AutoTarget(ModuleState.Forward)));
        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Pickup, new AutoTarget(ModuleState.Reverse)));
    }
}
