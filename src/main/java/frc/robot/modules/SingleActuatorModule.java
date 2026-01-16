package frc.robot.modules;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry; // added
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.action.ActionPose;


public class SingleActuatorModule implements RobotModule {

    Relay relay;
    String moduleID;
    Value currentValue = Value.kOff;
    Boolean invert;
    ModuleController controller;

    ActionPose targetPose;
    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    double startedAt = 0;
    double holdTime = 200;
    ModuleState previousState = ModuleState.Off;

    NetworkTable myTable;
    // Cached NT entries
    private NetworkTableEntry invertEntry;
    private NetworkTableEntry currentValueEntry;
    private NetworkTableEntry targetPoseEntry;

    public SingleActuatorModule(String ModuleID, Relay Relay, Boolean Invert) {
        moduleID = ModuleID;
        relay = Relay;
        invert = Invert;
    }

    @Override
    public void Initialize() {
        myTable = NetworkTableInstance.getDefault().getTable(moduleID);

        // instantiate entries
        invertEntry = myTable.getEntry("invert");
        currentValueEntry = myTable.getEntry("currentValue");
        targetPoseEntry = myTable.getEntry("targetPose");

        // set initial values
        invertEntry.setBoolean(invert);
    }

    public void EvaluateTargetPose() {
        if (targetPose != null) {
            var targetState = targetPose.target.State;

            if (startedAt == 0) {
                if (previousState == targetState) {
                    // we're already there, abandon target and move on
                    System.out.println("SingleActuatorModule: already in target state, abandoning target");
                    AbandonTarget();
                    return;
                }
                System.out.printf("SingleActuatorModule: new target state %s; previous target state %s\n", targetState, previousState);
                startedAt = System.currentTimeMillis();
            } else {
                // TODO: the timer shouldn't be built in here, maybe.  This is delaying the auto start time.
                if (System.currentTimeMillis() - startedAt > holdTime) {
                    AbandonTarget();
                    return;
                }
            }
            
            // determine what to do based on target pose - position X > 0 = forward, < 0 = reverse, 0 = off
            switch (targetState) {
                case Off:
                    previousState = ModuleState.Off;
                    AbandonTarget();
                    break;
                case Forward:
                    previousState = ModuleState.Forward;
                    ApplyValue(true);
                    break;
                case Reverse:
                    previousState = ModuleState.Reverse;
                    ApplyInverse(true);
                    break;
            }
        }
    }

    @Override
    public void ProcessState(boolean isAuto) {
        EvaluateTargetPose();

        relay.set(currentValue);
        // myTable.getEntry("currentValue").setString(currentValue.toString());
        currentValueEntry.setString(currentValue.toString());
        
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
    public void SetTargetActionPose(String Group, String Location, int LocationIndex, String Position,
        String Action) {
        var targetPose = GetActionPose(Group, Location, LocationIndex, Position, Action);
        if (targetPose != null) {
            // myTable.getEntry("targetPose").setString(String.format(...));
            targetPoseEntry.setString(String.format("%s %s %d %s %s", Group, Location, LocationIndex, Position, Action));

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
        // myTable.getEntry("targetPose").setString("none");
        targetPoseEntry.setString("none");
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
    public ActionPose GetActionPose(String group, String location, int locationIndex, String position,
        String action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || "any".equalsIgnoreCase(pose.group))
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || "any".equalsIgnoreCase(pose.location))
                && (pose.position == position || "any".equalsIgnoreCase(pose.position))
                && (pose.action == action || "any".equalsIgnoreCase(pose.action))
            ) {
                System.out.printf("%s GetActionPose: Matched %s %d %s %s %s\n", moduleID, pose.group, pose.location, pose.locationIndex, pose.position, pose.action);
                return pose;
            }
        }

        return null;      
    }
    
}
