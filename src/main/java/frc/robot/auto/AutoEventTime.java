package frc.robot.auto;

import java.util.function.Consumer;

// AutoEventTime represents an auto event that occurs based on a time trigger.
public class AutoEventTime implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;
    AutoController autoController;

    long milliseconds;

    EventType eventType;
    
    boolean boolValue;
    Consumer<Boolean> boolEvent;

    double doubleValue;
    Consumer<Double> doubleEvent;

    Runnable voidEvent;

    AutoSequence autoEvent;
    
    public AutoEventTime(String Label, boolean Parallel, long Milliseconds, EventType EventType, AutoController AutoController) {
        label = Label;
        parallel = Parallel;
        milliseconds = Milliseconds;
        autoController = AutoController;

        eventType = EventType;
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
            case SetTarget:
            case AwaitTarget:
                // not implemented for Time Event Type
                break;
        }
        complete = true;
    }

    public long GetMilliseconds() {
        return milliseconds;
    }

    public TriggerType GetTriggerType() {
        return TriggerType.Time;
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
