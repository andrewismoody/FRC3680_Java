package frc.robot.z2026;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.misc.Utility;
import frc.robot.modules.DriveModule;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleMotorModule;
import frc.robot.z2026.action.Location;
import frc.robot.z2026.action.Position;

public class ActionPoses {
    public static void Initialize(
        ModuleController moduleController
    ) {
        var swerveDriveModule = (DriveModule) moduleController.GetDriveModule();
        var shooter = (SingleMotorModule) moduleController.GetModule("shooter");
        var feeder = (SingleMotorModule) moduleController.GetModule("feeder");

        // start, (we don't know where we are yet, so rotate a specific angle to face a tag)
        // get facing direction
        var startRotation = swerveDriveModule.GetPositionerOffset().getRotation().toRotation2d();
        swerveDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Start.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Utility.getLookat(Constants.getMyStartPose().getTranslation().toTranslation2d(), Constants.getFieldPosition(Group.Any, Location.Interest, 1).toTranslation2d()).minus(startRotation))));

        // this spits out coordinates in format for python plotting
        var outputFormatter = "(\"%s %d %d %d %s\", %f, %f),\n";
        for (int i = 1; i <= 10; i++) {
            // waypoint i, Lookat Hub
            var pose = new ActionPose(Group.Travel, Location.Waypoint.getValue(), i, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, i), Constants.getFieldPosition(Group.Any, Location.Interest, 1)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        for (int i = 1; i <= Constants.HubIndexToTag.size(); i++) {
            var HubTag = Constants.HubIndexToTag.get(i);
            // get perpendicular direction facing toward
            var rotation = swerveDriveModule.GetPositionerOffset().getRotation().toRotation2d();
            rotation = Rotation2d.kCW_90deg.minus(rotation);

            // align scoring i, match HubTag rotation
            var pose = new ActionPose(Group.Align, Location.Hub.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Hub, i), Constants.getKnownRotation(Group.Any, Location.Tag, HubTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
                // approach scoring i, match HubTag rotation
            pose = new ActionPose(Group.Approach, Location.Hub.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Approach, Location.Hub, i), Constants.getKnownRotation(Group.Any, Location.Tag, HubTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
                // scoring i, match HubTag rotation
            pose = new ActionPose(Group.Score, Location.Hub.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Hub, i), Constants.getKnownRotation(Group.Any, Location.Tag, HubTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        for (int i = 1; i <= Constants.OutpostIndexToTag.size(); i++) {
            var OutpostTag = Constants.OutpostIndexToTag.get(i);
            // get perpendicular direction facing away
            var rotation = swerveDriveModule.GetPositionerOffset().getRotation().toRotation2d();
            rotation = Rotation2d.kCCW_90deg.minus(rotation);

            // align Outpost i, match OutpostTag rotation
            var pose = new ActionPose(Group.Align, Location.Outpost.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Outpost, i), Constants.getKnownRotation(Group.Any, Location.Tag, OutpostTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));

            // Outpost i, match OutpostTag rotation
            pose = new ActionPose(Group.Pickup, Location.Outpost.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Outpost, i), Constants.getKnownRotation(Group.Any, Location.Tag, OutpostTag).plus(rotation)));
            swerveDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        // elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.36)));
        // elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.55)));
        // elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        var shooterRpm = -3000; // TODO: tune this
        shooter.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Feed, new AutoTarget(shooterRpm)));
        shooter.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.None, new AutoTarget(0.0)));
        
        var feederRpm = 100; // TODO: tune this
        feeder.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Release, new AutoTarget(feederRpm)));
        feeder.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.None, new AutoTarget(0.0)));
    }
}
