package frc.robot.z2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static void Initialize(Transform3d redStartTransform, SwerveDriveModule swerveDriveModule, SingleMotorModule elevator, SingleActuatorModule slide) {
        // Position Targets
        var Waypoint1 = new Translation3d(2.65, 6.45, 0);
        var Waypoint11 = new Translation3d(6.35, 6.65, 0);
        var Waypoint12 = new Translation3d(4.47, 7.45, 0);
        var Scoring1 = new Translation3d(2.63, 5.66, 0);
        var Pickup1 = new Translation3d(1.15, 7.08, 0);

        // Lookat Targets
        var Tag1 = new Translation3d(0.85, 7.40, 0);
        var ReefCenter = new Translation3d(4.49, 4.03, 0);

        var StartRotation = new Rotation2d(2.75);

        if (Utility.IsRedAlliance()) {
            Waypoint1 = Waypoint1.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
            Waypoint11 = Waypoint11.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
            Waypoint12 = Waypoint12.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
            Scoring1 = Scoring1.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
            Pickup1 = Pickup1.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
            Tag1 = Tag1.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());
            ReefCenter = ReefCenter.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation());

            StartRotation = StartRotation.plus(redStartTransform.getRotation().toRotation2d());
        }

        // start, 240 (we don't know where we are yet, so rotate a specific angle to face a tag)
        swerveDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(StartRotation)));
        // waypoint 11, Lookta reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), 11, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint11, ReefCenter)));
        // waypoint 12, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), 12, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint12, ReefCenter)));
        // waypoint 1, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Travel, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint1, ReefCenter)));
        // scoring 1, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Align, Location.Reef.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Scoring1, ReefCenter)));

        // scoring 1, no movement, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(ReefCenter, true)));

        // TODO: seek tag breaks this - will have to hard code a rotation like start to switch lookat targets - or fix seek tag
        // waypoint 1, 126 -- should this be hard-coded to a rotation or will LookAt work as long as we don't "clear" our position?
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint1, Tag1)));
        // loading 1, 126
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Pickup1, Tag1)));

        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.36)));
        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.55)));
        elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Drop, new AutoTarget(ModuleState.Forward)));
        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Pickup, new AutoTarget(ModuleState.Reverse)));
    }
}
