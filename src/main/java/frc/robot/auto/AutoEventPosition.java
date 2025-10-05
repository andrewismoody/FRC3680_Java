package frc.robot.auto;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.modules.DriveModule;

// AutoEventPosition represents an auto event that is triggered when the drive module reaches a specific pose.
public class AutoEventPosition implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;
    AutoController autoController;

    Pose3d target;

    EventType eventType;

    boolean boolValue;
    Consumer<Boolean> boolEvent;

    double doubleValue;
    Consumer<Double> doubleEvent;

    Runnable voidEvent;

    AutoSequence autoEvent;

    boolean isNearby = false;
    boolean wasNearby = false;
    double positionTolerance = 0.1; // meters
    double rotationTolerance = 5.0; // degrees

    DriveModule driveModule;

    public AutoEventPosition(String Label, Boolean Parallel, Pose3d Target, EventType EventType, AutoController AutoController, DriveModule DriveModule) {
        label = Label;
        parallel = Parallel;
        eventType = EventType;
        autoController = AutoController;
        driveModule = DriveModule;

        target = Target;
    }

    public void Run() {
        wasNearby = isNearby;
        isNearby = isNearby(driveModule.GetPosition(), target, positionTolerance, rotationTolerance);
        
        if (isNearby) {
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
                default:
                    // auto-complete if parameters aren't correct
                    complete = true;
                    break;
            }
        }

        if (!isNearby && wasNearby) {
            // we were nearby, but now we're not - so reset the event
            complete = true;
        }
    }

    boolean isNearby(Pose3d Position, Pose3d Target, double PositionTolerance, double AngleTolerance) {
        if (Math.abs(Position.getX() - Target.getX()) > PositionTolerance)
            return false;

        if (Math.abs(Position.getY() - Target.getY()) > PositionTolerance)
            return false;

        if (Math.abs(Position.getRotation().getZ() - Target.getRotation().getZ()) > AngleTolerance)
            return false;

        return true;
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
