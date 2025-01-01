package frc.robot.auto;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Translation3d;

public class AutoEvent {
    public enum EventType {
        Time,
        Position
    }

    public boolean Complete;
    public EventType eventType;
    public Translation3d Position;
    public long Milliseconds;
    public Consumer<Double> DoubleEvent;
    public Consumer<Boolean> BoolEvent;
    public double DoubleValue;
    public boolean BoolValue;
    public String label;
}
