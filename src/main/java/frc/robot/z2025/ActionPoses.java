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
        // left start, 240
        swerveDriveModule.AddActionPose(new ActionPose(Group.Start, Location.Barge.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(9.271, 7.79, 0), new Rotation2d(3.654))));
        // waypoint 11, 240
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Waypoint.getValue(), 11, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(5.8, 5.75, 0), new Rotation2d(3.654))));
        // waypoint 12, 240
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Waypoint.getValue(), 12, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(4.47, 6.45, 0), new Rotation2d(4.7))));
        // waypoint 1, 300
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(3.15, 5.75, 0), new Rotation2d(5.22))));
        // reef 1, 300
        swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(3.75, 5.05, 0), new Rotation2d(5.22))));
        // waypoint 1, 126
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Waypoint.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(3.15, 5.75, 0), new Rotation2d(2.1924))));
        // loading 1, 126
        swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral.getValue(), 1, Position.Any.getValue(), Action.Any, new AutoTarget(new Translation3d(1.15, 7.08, 0), new Rotation2d(2.1924))));

        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Lower.getValue(), Action.Any, new AutoTarget(0.28)));
        elevator.AddActionPose(new ActionPose(Group.Score, Location.Any.getValue(), -1, Position.Middle.getValue(), Action.Any, new AutoTarget(1.14)));
        elevator.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Trough.getValue(), Action.Any, new AutoTarget(0.0)));

        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Drop, new AutoTarget(ModuleState.Forward)));
        slide.AddActionPose(new ActionPose(Group.Any, Location.Any.getValue(), -1, Position.Any.getValue(), Action.Pickup, new AutoTarget(ModuleState.Reverse)));
    }
}
