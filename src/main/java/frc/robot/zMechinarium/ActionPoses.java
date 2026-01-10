package frc.robot.zMechinarium;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.misc.Utility;
import frc.robot.modules.DriveModule;
import frc.robot.modules.ModuleState;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.zMechinarium.action.Location;
import frc.robot.zMechinarium.action.Position;

public class ActionPoses {
    public static void Initialize(DriveModule swerveDriveModule, SingleMotorModule elevator, SingleActuatorModule slide) {
        // start, (we don't know where we are yet, so rotate a specific angle to face a tag)
        // get facing direction
        var startRotation = swerveDriveModule.GetPositionerOffset().getRotation().toRotation2d();
        swerveDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Utility.getLookat(Constants.getMyStartPose().getTranslation().toTranslation2d(), Constants.getFieldPosition(Group.Any, Location.Interest, 1).toTranslation2d()).minus(startRotation))));

        // this spits out coordinates in format for python plotting
        var outputFormatter = "(\"%s %d %d %d %s\", %f, %f),\n";
        for (int i = 1; i <= 12; i++) {
            // waypoint i, Lookat reef
            var pose = new ActionPose(Group.Travel, Location.Waypoint.getValue(), i, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, i), Constants.getFieldPosition(Group.Any, Location.Interest, 1)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        for (int i = 1; i <= 12; i++) {
            var reefTag = Constants.reefIndexToTag.get(i);
            // get perpendicular direction facing toward
            var rotation = swerveDriveModule.GetPositionerOffset().getRotation().toRotation2d();
            rotation = Rotation2d.kCW_90deg.minus(rotation);

            // align left scoring i, match reefTag rotation
            var pose = new ActionPose(Group.AlignLeft, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.AlignLeft, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
                // approach left scoring i, match reefTag rotation
            pose = new ActionPose(Group.ApproachLeft, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.ApproachLeft, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
            // align right scoring i, match reefTag rotation
            pose = new ActionPose(Group.AlignRight, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.AlignRight, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
            // approach right scoring i, match reefTag rotation
            pose = new ActionPose(Group.ApproachRight, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.ApproachRight, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
                // scoring i, match reefTag rotation
            pose = new ActionPose(Group.Score, Location.Reef.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Reef, i), Constants.getKnownRotation(Group.Any, Location.Tag, reefTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        for (int i = 1; i <= 2; i++) {
            var coralTag = Constants.coralIndexToTag.get(i);
            // get perpendicular direction facing away
            var rotation = swerveDriveModule.GetPositionerOffset().getRotation().toRotation2d();
            rotation = Rotation2d.kCCW_90deg.minus(rotation);

            // align coral i, match coralTag rotation
            var pose = new ActionPose(Group.Align, Location.Coral.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Coral, i), Constants.getKnownRotation(Group.Any, Location.Tag, coralTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));

            // coral i, match coralTag rotation
            pose = new ActionPose(Group.Pickup, Location.Coral.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Coral, i), Constants.getKnownRotation(Group.Any, Location.Tag, coralTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.36)));
        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.55)));
        elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Drop, new AutoTarget(ModuleState.Forward)));
        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Pickup, new AutoTarget(ModuleState.Reverse)));
    }
}
