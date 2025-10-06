package frc.robot.modules;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;


public class SingleActuatorModule implements RobotModule {

    Relay relay;
    String moduleID;
    Value currentValue = Value.kOff;
    Boolean invert;
    ModuleController controller;

    ActionPose targetPose;
    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    double startedAt = 0;
    double holdTime = 2000;

    NetworkTable myTable;

    public SingleActuatorModule(String ModuleID, Relay Relay, Boolean Invert) {
        moduleID = ModuleID;
        relay = Relay;
        invert = Invert;
    }

    @Override
    public void Initialize() {
        myTable = NetworkTableInstance.getDefault().getTable(moduleID);

        myTable.getEntry("invert").setBoolean(invert);
    }

    public void EvaluateTargetPose() {
        if (targetPose != null) {
            if (startedAt == 0) {
                startedAt = System.currentTimeMillis();
            } else {
                if (System.currentTimeMillis() - startedAt > holdTime) {
                    AbandonTarget();
                    return;
                }
            }
            
            // determine what to do based on target pose - position X > 0 = forward, < 0 = reverse, 0 = off
            var targetState = targetPose.target.State;
            switch (targetState) {
                case Off:
                AbandonTarget();
                break;
                case Forward:
                    ApplyValue(true);
                    break;
                case Reverse:
                    ApplyInverse(true);
                    break;
            }
        }
    }

    @Override
    public void ProcessState(boolean isAuto) {
        EvaluateTargetPose();

        relay.set(currentValue);
        myTable.getEntry("currentValue").setString(currentValue.toString());
        
        // reset to off after applying - this ensures that if not actively set, the module is off and doesn't stay on inadvertently
        // should we be always reverse (down) unless requested to be forward (up)? this would prevent accidental release of the game piece, but might incur battery usage
        currentValue = Value.kOff;
    }

    @Override
    public void ApplyValue(boolean value) {
        if (value) {
            if (invert) {
                currentValue = Value.kReverse;
            } else {
                currentValue = Value.kForward;
            }
        }
    }

    @Override
    public void ApplyInverse(boolean value) {
        if (value) {
            if (invert) {
                currentValue = Value.kForward;
            } else {
                currentValue = Value.kReverse;
            }
        }
    }

    @Override
    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    @Override
    public String GetModuleID() {
        return moduleID;
    }

    @Override
    public void SetTargetActionPose(Group Group, int Location, int LocationIndex, int Position,
            Action Action) {
        var targetPose = GetActionPose(Group, Location, LocationIndex, Position, Action);
        if (targetPose != null) {
            myTable.getEntry("targetPose").setString(String.format("%s %s %d %s %s", Group, Location, LocationIndex, Position, Action));

            this.targetPose = targetPose;
            startedAt = 0;
        }
    }

    @Override
    public void SetTargetActionPose(ActionPose ActionPose) {
        SetTargetActionPose(ActionPose.group, ActionPose.location, ActionPose.locationIndex, ActionPose.position, ActionPose.action);
    }

    public ActionPose GetTarget() {
        return targetPose;
    }

    public void AbandonTarget() {
        myTable.getEntry("targetPose").setString("none");
        targetPose = null;
    }

    @Override
    public Pose3d GetPosition() {
        return new Pose3d(new Translation3d(currentValue == Value.kOff ? 0 : currentValue == Value.kForward ? 1 : -1, 0, 0), new Rotation3d());
    }

    @Override
    public void AddActionPose(ActionPose NewPose) {
        if (GetActionPose(NewPose) == null) {
            actionPoses.add(NewPose);
        }
    }

    public ActionPose GetActionPose(ActionPose newAction) {
        return GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action);
    }

    @Override
    public ActionPose GetActionPose(Group group, int location, int locationIndex, int position,
            Action action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || pose.group == Group.Any)
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || pose.location == -1)
                && (pose.position == position || pose.position == -1)
                && (pose.action == action || pose.action == Action.Any)
            ) {
                return pose;
            }
        }

        return null;      
    }
    
}
