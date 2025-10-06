package frc.robot.z2025;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.modules.ModuleState;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.SwerveDriveModule;
import frc.robot.z2025.action.Location;
import frc.robot.z2025.action.Position;

public class ActionPoses {
    public static void Initialize(SwerveDriveModule swerveDriveModule, SingleMotorModule elevator, SingleActuatorModule slide) {
        // Position Targets
        var Waypoint1 = new Translation3d(3.15, 5.75, 0);
        var Waypoint11 = new Translation3d(5.8, 5.75, 0);
        var Waypoint12 = new Translation3d(4.47, 6.45, 0);
        var Scoring1 = new Translation3d(3.75, 5.05, 0);
        var Loading1 = new Translation3d(1.15, 7.08, 0);

        // Lookat Targets
        var Tag1 = new Translation3d(0.85, 7.40, 0);
        var ReefCenter = new Translation3d(4.49, 4.03, 0);

        // waypoint 11, 240 (we don't know where we are yet, so rotate a specific angle to face a tag)
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Waypoint.getValue(), 11, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint11, new Rotation2d(3.654))));
        // waypoint 12, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Waypoint.getValue(), 12, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint12, ReefCenter)));
        // waypoint 1, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint1, ReefCenter)));
        // scoring 1, Lookat reef
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Scoring1, ReefCenter)));
        // waypoint 1, 126 -- should this be hard-coded to a rotation or will LookAt work as long as we don't "clear" our position?
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Waypoint1, Tag1)));
        // loading 1, 126
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(Loading1, Tag1)));

        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.28)));
        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.14)));
        elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Drop, new AutoTarget(ModuleState.Forward)));
        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Pickup, new AutoTarget(ModuleState.Reverse)));
    }
}
