package frc.robot.auto;

import java.util.function.Consumer;

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
                voidEvent.run();
                break;
            case Boolean:
                boolEvent.accept(boolValue);
                break;
            case Double:
                doubleEvent.accept(doubleValue);
                break;
            case Auto:
                autoController.AddSequence(autoEvent);
                break;
            case Adaptive:
                // not implemented - shouldn't be used for Time triggers, as there's no target to meet.
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
