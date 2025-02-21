package frc.robot.auto;

import java.util.function.Consumer;

import frc.robot.RobotModule;
import frc.robot.action.ActionPose;

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
                break;
            case Boolean:
                if (boolEvent != null)
                    boolEvent.accept(boolValue);
                break;
            case Double:
                if (doubleEvent != null)
                    doubleEvent.accept(doubleValue);
                break;
            case Auto:
                if (autoEvent != null)
                    autoController.AddSequence(autoEvent);
                break;
            case Adaptive:
                if (targetModule != null && target != null)
                    targetModule.SetTargetActionPose(target.action, target.primary, target.secondary);
                break;
        }
        complete = true;
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
