package frc.robot.modules;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;

public class DualMotorModule implements RobotModule {
    public enum MotorSide {
        Left,
        Right
    }
    
    String moduleID;
    SingleMotorModule leftDriveMotor;
    SingleMotorModule rightDriveMotor;
    ModuleController controller;

    public boolean debug = false;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    ActionPose targetPose;
    MotorSide primaryMotor = MotorSide.Left;

    ArrayList<Consumer<Boolean>> buttonMappedTargets = new ArrayList<Consumer<Boolean>>();
    ArrayList<Consumer<Boolean>> buttonMappedPoses = new ArrayList<Consumer<Boolean>>();

    NetworkTable myTable;
    // Cached entries
    private NetworkTableEntry ntTargetPose;

    public DualMotorModule(String ModuleID, SingleMotorModule LeftDriveMotor, SingleMotorModule RightDriveMotor) {
        moduleID = ModuleID;
        leftDriveMotor = LeftDriveMotor;
        rightDriveMotor = RightDriveMotor;
    }

    public void Initialize() {
        // Initialize table and cache entries
        myTable = NetworkTableInstance.getDefault().getTable(moduleID);
        ntTargetPose = myTable.getEntry("targetPose");

        leftDriveMotor.Initialize();
        rightDriveMotor.Initialize();
    }

    public void AddActionPose(ActionPose newpose) {
        if (GetActionPose(newpose) == null) {
            actionPoses.add(newpose);
        }

        var inverted = new ActionPose(newpose.group, newpose.location, newpose.locationIndex, newpose.position, newpose.action, new AutoTarget(-newpose.target.Distance));

        if (leftDriveMotor.invert)
            leftDriveMotor.AddActionPose(inverted);
        else
            leftDriveMotor.AddActionPose(newpose);

        if (rightDriveMotor.invert)
            rightDriveMotor.AddActionPose(inverted);
        else
            rightDriveMotor.AddActionPose(newpose);
    }

    public ActionPose GetActionPose(ActionPose newAction) {
        return GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action);
    }

    public ActionPose GetActionPose(Group group, int location, int locationIndex, int position, Action action) {
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

    public ActionPose GetTarget() {
        return targetPose;
    }

    @Override
    public void ApplyInverse(boolean isPressed) {
        leftDriveMotor.ApplyInverse(isPressed);
        rightDriveMotor.ApplyInverse(isPressed);
    }

    @Override
    public void ApplyValue(boolean isPressed) {
        leftDriveMotor.ApplyValue(isPressed);
        rightDriveMotor.ApplyValue(isPressed);  
    }
    
    public void ProcessState(boolean isAuto) {
        leftDriveMotor.ProcessState(isAuto);
        rightDriveMotor.ProcessState(isAuto);

        if (leftDriveMotor.targetPose == null && rightDriveMotor.targetPose == null)
            targetPose = null;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;

        leftDriveMotor.SetController(Controller);
        rightDriveMotor.SetController(Controller);
    }   

    public String GetModuleID() {
        return moduleID;
    }

    public void SetTargetActionPose(Group group, int location, int locationIndex, int position, Action action) {
        var targetPose = GetActionPose(group, location, locationIndex, position, action);
        if (targetPose != null) {
            ntTargetPose.setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));

            this.targetPose = targetPose;
        }
        
        leftDriveMotor.SetTargetActionPose(group, location, locationIndex, position, action);
        rightDriveMotor.SetTargetActionPose(group, location, locationIndex, position, action);
    } 
    
    public void OverrideTargetActionPose(ActionPose newpose) {
        targetPose = newpose;

        var inverted = new ActionPose(newpose.group, newpose.location, newpose.locationIndex, newpose.position, newpose.action, new AutoTarget(-newpose.target.Distance));

        if (leftDriveMotor.invert)
            leftDriveMotor.OverrideTargetActionPose(inverted);
        else
            leftDriveMotor.OverrideTargetActionPose(newpose);

        if (rightDriveMotor.invert)
            rightDriveMotor.OverrideTargetActionPose(inverted);
        else
            rightDriveMotor.OverrideTargetActionPose(newpose);
    }

    public void SetTargetActionPose(ActionPose actionPose) {
        SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
    }   

    public Pose3d GetPosition() {
        switch (primaryMotor) {
            case Right:
                return rightDriveMotor.GetPosition();
            case Left:
            default:
                return leftDriveMotor.GetPosition();
        }
    }

    // AddButtonMappedPose adds a position to the array and returns a reference to the function
    // always make a new instance, don't reuse button mappings as they will stomp on each other
    public Consumer<Boolean> AddButtonMappedPose(ActionPose pose) {    
        var setTarget = new Consumer<Boolean>() {
            boolean wasPressed = false;

            @Override
            public void accept(Boolean pressed) {
                if (pressed && !wasPressed) {
                    SetTargetActionPose(pose);
                }
                wasPressed = pressed;
            }
        };
        buttonMappedPoses.add(setTarget);

        return setTarget;
    }

    // GetButtonMappedPose finds the function by index and returns it. Returns null if not found.
    // always make a new instance, don't reuse button mappings as they will stomp on each other
    public Consumer<Boolean> GetButtonMappedPose(int index) {
        if (index >= 0 && index < buttonMappedPoses.size()) {
            return buttonMappedPoses.get(index);
        }

        return null;
    }

    // AddButtonMappedTarget adds a position to the array and returns a reference to the function
    // always make a new instance, don't reuse button mappings as they will stomp on each other
    public Consumer<Boolean> AddButtonMappedTarget(double position) {
        var setTarget = new Consumer<Boolean>() {
            boolean wasPressed = false;

            @Override
            public void accept(Boolean pressed) {
                // multiple mapped consumers defeat this logic as one is pressed and one is not.
                if (pressed && !this.wasPressed) {
                    OverrideTargetActionPose(new ActionPose(Group.Any, -1, -1, -1, Action.Any, new AutoTarget(position)));
                }
                this.wasPressed = pressed;
            }
        };
        buttonMappedTargets.add(setTarget);

        return setTarget;
    }

    // GetButtonMappedTarget finds the function by index and returns it. Returns null if not found.
    public Consumer<Boolean> GetButtonMappedTarget(int index) {
        if (index >= 0 && index < buttonMappedTargets.size()) {
            return buttonMappedTargets.get(index);
        }

        return null;
    }

    public void SetNoPose(boolean isPressed) {
        if (isPressed) {
            AbandonTarget();
        }
    }

    public void AbandonTarget() {
        targetPose = null;

        leftDriveMotor.AbandonTarget();
        rightDriveMotor.AbandonTarget();
    }
}
