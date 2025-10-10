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
    boolean hasStarted = false;
    
    // AutoEventTime will trigger an event for the specified number of milliseconds.  If an AutoEvent is supplied, it will be triggered at the end of the time period.
    public AutoEventTime(String Label, boolean Parallel, long Milliseconds, EventType EventType, AutoController AutoController) {
        label = Label;
        parallel = Parallel;
        // make 60 the minimum to ensure it always fires
        milliseconds = Milliseconds < 60 ? 60 : Milliseconds;
        autoController = AutoController;

        eventType = EventType;
    }

    public void SetDoubleEvent(double Value, Consumer<Double> Event) {
        doubleValue = Value;
        doubleEvent = Event;
    }

    public void SetBoolEvent(boolean Value, Consumer<Boolean> Event) {
        boolValue = Value;
        boolEvent = Event;
    }

    public void SetAutoEvent(AutoSequence AutoEvent) {
        autoEvent = AutoEvent;
    }

    public void SetVoidEvent(Runnable Event) {
        voidEvent = Event;
    }

    public boolean HasStarted() {
        return hasStarted;
    }

    public void Run() {
        if (!hasStarted)
            hasStarted = true;

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
                // only trigger auto event at the end of the time period (setcomplete=true)
            case SetTarget:
            case AwaitTarget:
            case None:
                // not implemented for Time Event Type
                break;
        }
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
        // trigger the next autoevent at the end of the time period
        if (Complete && autoEvent != null)
            autoController.AddSequence(autoEvent);

        complete = Complete;
        System.out.printf("AutoEventTime %s isComplete: %b\n", label, complete);
    }

    public String GetLabel() {
        return label;
    }

    public boolean IsParallel() {
        return parallel;
    }
}
