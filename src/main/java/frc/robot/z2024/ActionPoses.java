package frc.robot.z2024;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.misc.Utility;
import frc.robot.modules.DriveModule;
import frc.robot.modules.DualMotorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.z2024.action.Location;
import frc.robot.z2024.action.Position;

public class ActionPoses {
    public static void Initialize(DriveModule diffDriveModule, DualMotorModule shoot, SingleMotorModule feed, SingleMotorModule pickup, DualMotorModule lift) {
        // this spits out coordinates in format for python plotting
        var outputFormatter = "(\"%s %d %d %d %s\", %f, %f),\n";
        var outputRotFormatter = "(\"%s %d %d %d %s\", %f, %f), # rotation: %f\n";

        // start, (we don't know where we are yet, so rotate a specific angle to face a tag)
        // get facing direction
        var startRotation = diffDriveModule.GetPositionerOffset().getRotation().toRotation2d();
        diffDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Start.getValue(), 1, Position.Any.getValue(), Action.Any,
            new AutoTarget(Utility.getLookat(Constants.getMyStartPose().getTranslation().toTranslation2d(), Constants.getFieldPosition(Group.Any, Location.Interest, 1).toTranslation2d()).minus(startRotation))));
        var scorepose = new ActionPose(Group.Any, Location.Interest.getValue(), 2, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Interest, 2), Constants.getKnownRotation(Group.Any, Location.Interest, 2)));
        diffDriveModule.AddActionPose(scorepose);
        System.out.printf(outputFormatter, scorepose.group, scorepose.location, scorepose.locationIndex, scorepose.position, scorepose.action, Utility.metersToInches(scorepose.target.Position.getX()), Utility.metersToInches(scorepose.target.Position.getY()));

        for (int i = 1; i <= 6; i++) {
            // waypoint i, Lookat stage
            var pose = new ActionPose(Group.Travel, Location.Waypoint.getValue(), i, Position.Any.getValue(), Action.Any,
            new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Waypoint, i), Constants.getFieldPosition(Group.Any, Location.Interest, 1)));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        for (int i = 1; i <= 3; i++) {
            var stageTag = Constants.stageIndexToTag.get(i);
            // get perpendicular direction facing toward
            var rotation = Rotation2d.kCCW_90deg;

            // align left scoring i, match reefTag rotation
            var pose = new ActionPose(Group.AlignLeft, Location.Stage.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.AlignLeft, Location.Stage, i), Constants.getKnownRotation(Group.Any, Location.Tag, stageTag).plus(rotation)));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
                // approach left scoring i, match reefTag rotation
            pose = new ActionPose(Group.ApproachLeft, Location.Stage.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.ApproachLeft, Location.Stage, i), Constants.getKnownRotation(Group.Any, Location.Tag, stageTag).plus(rotation)));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
            // align right scoring i, match reefTag rotation
            pose = new ActionPose(Group.AlignRight, Location.Stage.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.AlignRight, Location.Stage, i), Constants.getKnownRotation(Group.Any, Location.Tag, stageTag).plus(rotation)));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
            // approach right scoring i, match reefTag rotation
            pose = new ActionPose(Group.ApproachRight, Location.Stage.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.ApproachRight, Location.Stage, i), Constants.getKnownRotation(Group.Any, Location.Tag, stageTag).plus(rotation)));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
                // scoring i, match reefTag rotation
            pose = new ActionPose(Group.Score, Location.Stage.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Stage, i), Constants.getKnownRotation(Group.Any, Location.Tag, stageTag).plus(rotation)));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()));
        }

        for (int i = 1; i <= 2; i++) {
            var speakerTag = Constants.speakerIndexToTag.get(i);
            // get perpendicular direction facing away
            // don't adjust for positioner offset, as scoring is from back of robot
            var rotation = Rotation2d.kCW_90deg;
            var tagRotation = Constants.getKnownRotation(Group.Any, Location.Tag, speakerTag);
            tagRotation = tagRotation.plus(rotation);

            // align speaker i, match speakerTag rotation
            var pose = new ActionPose(Group.Align, Location.Speaker.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Speaker, i), tagRotation));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputRotFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()), pose.target.Orientation.getDegrees());

            // speaker i, match speakerTag rotation
            pose = new ActionPose(Group.Pickup, Location.Speaker.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Speaker, i), tagRotation));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputRotFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()), pose.target.Orientation.getDegrees());
        }

        for (int i = 1; i <= 2; i++) {
            var sourceTag = Constants.sourceIndexToTag.get(i);
            // get perpendicular direction facing away
            // don't adjust for positioner offset, as scoring is from back of robot
            var rotation = Rotation2d.kCW_90deg;
            var tagRotation = Constants.getKnownRotation(Group.Any, Location.Tag, sourceTag);
            tagRotation = tagRotation.plus(rotation);

            // align source i, match sourceTag rotation
            var pose = new ActionPose(Group.Align, Location.Source.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Align, Location.Source, i), tagRotation));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputRotFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()), pose.target.Orientation.getDegrees());

            // source i, match sourceTag rotation
            pose = new ActionPose(Group.Pickup, Location.Source.getValue(), i, Position.Any.getValue(), Action.Any,
                new AutoTarget(Constants.getFieldPosition(Group.Any, Location.Source, i), tagRotation));
            diffDriveModule.AddActionPose(pose);
            System.out.printf(outputRotFormatter, pose.group, pose.location, pose.locationIndex, pose.position, pose.action, Utility.metersToInches(pose.target.Position.getX()), Utility.metersToInches(pose.target.Position.getY()), pose.target.Orientation.getDegrees());
        }

        // shoot uses velocity targets, so it will try to reach the 28.8 m/s, then sustain that velocity until told otherwise
        shoot.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Upper.getValue(), Action.Any, new AutoTarget(28.8)));
        shoot.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(14.4)));
        shoot.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Ground.getValue(), Action.Any, new AutoTarget(0.0)));
    }
}
