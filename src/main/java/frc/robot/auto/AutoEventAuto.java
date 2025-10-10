package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation3d;

// AutoEventAuto represents a nested AutoSequence event within an autonomous routine.
public class AutoEventAuto implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;
    boolean hasStarted = false;

    Translation3d target;

    EventType eventType;

    AutoSequence autoEvent;

    public AutoEventAuto(String Label, Boolean Parallel, AutoSequence AutoEvent, EventType EventType) {
        label = Label;
        parallel = Parallel;
        eventType = EventType;

        autoEvent = AutoEvent;
    }

    public boolean HasStarted() {
        return hasStarted;
    }

    public void Run() {
        if (!hasStarted)
            hasStarted = true;

        complete = true;
    }

    public boolean IsFinished() {
        return autoEvent.IsFinished();
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
        System.out.printf("AutoEventTime %s isComplete: %b\n", label, complete);
    }

    public String GetLabel() {
        return label;
    }

    public boolean IsParallel() {
        return parallel;
    }
}
