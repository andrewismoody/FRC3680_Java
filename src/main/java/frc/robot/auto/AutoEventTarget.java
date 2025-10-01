package frc.robot.auto;

import java.util.function.Consumer;

import frc.robot.ModuleController;
import frc.robot.RobotModule;
import frc.robot.action.ActionPose;

// AutoEventTarget represents an auto event that attempts to achieve a specific position or pose.
public class AutoEventTarget implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;
    AutoController autoController;

    ActionPose target;

    EventType eventType;

    boolean boolValue;
    Consumer<Boolean> boolEvent;

    double doubleValue;
    Consumer<Double> doubleEvent;

    Runnable voidEvent;

    AutoSequence autoEvent;

    RobotModule targetModule;
    ModuleController moduleController;

    public AutoEventTarget(String Label, Boolean Parallel, ActionPose Target, EventType EventType, AutoController AutoController) {
        label = Label;
        parallel = Parallel;
        eventType = EventType;
        autoController = AutoController;

        target = Target;
    }

    public void Run() {
        switch (eventType) {
            case SetTarget:
                if (target != null) {
                    if (targetModule != null)
                        targetModule.SetTargetActionPose(target);
                    else if (moduleController != null)
                        moduleController.SetTargetActionPose(target);
                }
                complete = true;
                break;
            case AwaitTarget:
                if (targetModule == null && moduleController == null) {
                    complete = true;
                    break;
                }

                if (targetModule != null && targetModule.GetTarget() == null) {
                    complete = true;
                }

                if (moduleController != null && !moduleController.GetTarget()) {
                    complete = true;
                }
                break;
            default:
                complete = true;
                break;
        }
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
}
