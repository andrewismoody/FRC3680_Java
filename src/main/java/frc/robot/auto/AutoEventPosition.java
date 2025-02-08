package frc.robot.auto;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotModule;

public class AutoEventPosition implements AutoEvent {
    boolean complete;
    boolean parallel;
    String label;
    AutoController autoController;

    Translation3d target;

    EventType eventType;

    boolean boolValue;
    Consumer<Boolean> boolEvent;

    double doubleValue;
    Consumer<Double> doubleEvent;

    Runnable voidEvent;

    AutoSequence autoEvent;

    RobotModule targetModule; // for adaptive events
    Pose3d targetPose;

    public AutoEventPosition(String Label, Boolean Parallel, Translation3d Target, EventType EventType, AutoController AutoController) {
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
                if (targetModule != null && targetPose != null)
                    targetModule.ApproachTarget(targetPose);
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
