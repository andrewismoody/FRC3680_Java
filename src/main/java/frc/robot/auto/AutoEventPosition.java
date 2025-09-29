package frc.robot.auto;

import java.util.function.Consumer;

import frc.robot.RobotModule;
import frc.robot.action.ActionPose;

// AutoEventPosition represents an auto event that attempts to achieve a specific position or pose.
public class AutoEventPosition implements AutoEvent {
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

    RobotModule targetModule; // for adaptive events

    public AutoEventPosition(String Label, Boolean Parallel, ActionPose Target, EventType EventType, AutoController AutoController) {
        label = Label;
        parallel = Parallel;
        eventType = EventType;
        autoController = AutoController;

        target = Target;
    }

    public void Run() {
        switch (eventType) {
            case Void:
                if (voidEvent != null)
                    voidEvent.run();
                complete = true;
                break;
            case Boolean:
                if (boolEvent != null)
                    boolEvent.accept(boolValue);
                complete = true;
                break;
            case Double:
                if (doubleEvent != null)
                    doubleEvent.accept(doubleValue);
                complete = true;
                break;
            case Auto:
                if (autoEvent != null)
                    autoController.AddSequence(autoEvent);
                complete = true;
                break;
            case SetTarget:
                if (targetModule != null && target != null)
                    targetModule.SetTargetActionPose(target);
                complete = true;
                break;
            case AwaitTarget:
                if (targetModule == null) {
                    complete = true;
                    break;
                }
                if (targetModule.GetTarget() == null) {
                    complete = true;
                }
                break;
        }
    }

    public TriggerType GetTriggerType() {
        return TriggerType.Position;
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
