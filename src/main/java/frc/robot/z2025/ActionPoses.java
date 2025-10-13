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
        // start, (we don't know where we are yet, so rotate a specific angle to face a tag)
        swerveDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Utility.getLookat(Constants.getStartPose().getTranslation().toTranslation2d(), Constants.getFieldPosition(Group.Any, Location.Interest, 1).toTranslation2d()).minus(Rotation2d.fromDegrees(90)))));

        for (int i = 1; i <= 12; i++) {
            // waypoint i, Lookat reef
            swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, i), Constants.getFieldPosition(Group.Any, Location.Interest, 1))));
        }

        for (int i = 1; i <= 12; i++) {
            var reefTag = Constants.reefIndexToTag.get(i);

            // align scoring i, match reefTag rotation
            swerveDriveModule.AddActionPose(new ActionPose(Group.Align, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(Rotation2d.fromDegrees(180)))));
            // approach scoring i, match reefTag rotation
            swerveDriveModule.AddActionPose(new ActionPose(Group.Approach, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Approach, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(Rotation2d.fromDegrees(180)))));
            // scoring i, match reefTag rotation
            swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(Rotation2d.fromDegrees(180)))));
        }

        for (int i = 1; i <= 2; i++) {
            var coralTag = Constants.coralIndexToTag.get(i);

            // align coral i, match coralTag rotation
            swerveDriveModule.AddActionPose(new ActionPose(Group.Align, Location.Coral.getValue(), i, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Coral, i), Constants.getKnownRotation(Group.Any, Location.Tag, coralTag))));

            // coral i, match coralTag rotation
            swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Coral, i), Constants.getKnownRotation(Group.Any, Location.Tag, coralTag))));
        }

        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.36)));
        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.55)));
        elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Drop, new AutoTarget(ModuleState.Forward)));
        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Pickup, new AutoTarget(ModuleState.Reverse)));
    }
}
