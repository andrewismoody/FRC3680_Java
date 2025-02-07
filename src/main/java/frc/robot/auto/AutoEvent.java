package frc.robot.auto;

public interface AutoEvent {
    public enum TriggerType {
        Time,
        Position,
        Auto,
    }

    public enum EventType {
        Auto,
        Void,
        Boolean,
        Double,
        Adaptive, // the idea of this event type is to allow the target module to perform its own calculations to meet the target
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

}
