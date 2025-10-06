package frc.robot.auto;

import frc.robot.action.ActionPose;
import frc.robot.modules.ModuleController;
import frc.robot.modules.RobotModule;

// AutoEventTarget represents an auto event that attempts to achieve a specific position or pose.
public class AutoEventTarget implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;
    boolean hasSetTarget = false;
    AutoController autoController;

    ActionPose target;

    EventType eventType;

    RobotModule targetModule;
    ModuleController moduleController;

    public AutoEventTarget(String Label, Boolean Parallel, ActionPose Target, EventType EventType, AutoController AutoController) {
        label = Label;
        parallel = Parallel;
        eventType = EventType;
        autoController = AutoController;
        moduleController = autoController.GetModuleController();

        target = Target;
    }

    public void Run() {
        switch (eventType) {
            case SetTarget:
                if (target != null) {
                    SetTarget();
                }
                complete = true;
                break;
            case AwaitTarget:
                if (targetModule == null && moduleController == null) {
                    complete = true;
                    break;
                }

                // Allow setting and awaiting in same action
                if (!hasSetTarget) {
                    if (target != null) {
                        SetTarget();
                    } else {
                        complete = IsTargetReached();
                    }
                } else {
                    complete = IsTargetReached();
                }
                break;
            default:
                complete = true;
                break;
        }
    }

    public void SetTarget() {
        if (targetModule != null)
            targetModule.SetTargetActionPose(target);
        else if (moduleController != null)
            moduleController.SetTargetActionPose(target);
        hasSetTarget = true;
    }

    public boolean IsTargetReached() {
        if (targetModule != null)
            return targetModule.GetTarget() == null;
        else if (moduleController != null)
            return !moduleController.GetTarget();
        return false;
    }

    public TriggerType GetTriggerType() {
        return TriggerType.Target;
    }

    public EventType GetEventType() {
        return eventType;
    }

    public boolean IsComplete() {
        return complete;
    }

    public void SetComplete(boolean Complete) {
        complete = Complete;
    }

    public String GetLabel() {
        return label;
    }

    public boolean IsParallel() {
        return parallel;
    }

    public void SetTargetModule(RobotModule TargetModule) {
        targetModule = TargetModule;
    }

    public RobotModule GetTargetModule() {
        return targetModule;
    }

    public ModuleController GetModuleController() {
        return moduleController;
    }
}
