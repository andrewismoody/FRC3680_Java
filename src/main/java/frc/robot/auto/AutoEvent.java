package frc.robot.auto;

// AutoEvent is the interface that defines the structure for all autonomous events.
public interface AutoEvent {
    public enum TriggerType {
        Time,
        Position,
        Target,
        Auto,
    }

    public enum EventType {
        Auto,
        Void,
        Boolean,
        Double,
        SetTarget, // the idea of this event type is to allow the target module to perform its own calculations to meet the target
        AwaitTarget,
        None,
    }

    public enum ModuleType {
        Drive,
        Actuator,
    }

    public TriggerType GetTriggerType();
    public EventType GetEventType();
    public boolean IsComplete();
    public void SetComplete(boolean Complete);
    public String GetLabel();
    public boolean IsParallel();
    public void Run();
    public boolean HasStarted();
}
