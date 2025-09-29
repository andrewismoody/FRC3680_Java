package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation3d;

// AutoEventAuto represents a nested AutoSequence event within an autonomous routine.
public class AutoEventAuto implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;

    Translation3d target;

    EventType eventType;

    AutoSequence autoEvent;

    public AutoEventAuto(String Label, Boolean Parallel, AutoSequence AutoEvent, EventType EventType) {
        label = Label;
        parallel = Parallel;
        eventType = EventType;

        autoEvent = AutoEvent;
    }

    public void Run() {
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
    }

    public String GetLabel() {
        return label;
    }

    public boolean IsParallel() {
        return parallel;
    }
}
