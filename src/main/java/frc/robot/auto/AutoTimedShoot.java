package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.auto.AutoEvent.EventType;

public class AutoTimedShoot extends AutoController {
    public AutoTimedShoot(ModuleController modules) {
        super(modules);

        AutoEvent event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event.DoubleValue = -1.0;
        event.Milliseconds = 0;
        event.label = "Move Backward";
        AddEvent(event);

        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event.DoubleValue = 0.0;
        event.Milliseconds = 2000;
        event.label = "Stop";
        AddEvent(event);

        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.BoolEvent = modules.GetModule("ejector")::ProcessState;
        event.BoolValue = true;
        event.Milliseconds = 0;
        event.label = "Start Shooter";
        AddEvent(event);

        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.BoolEvent = modules.GetModule("ejector")::ProcessState;
        event.BoolValue = false;
        event.Milliseconds = 2000;
        event.label = "Stop Shooter";
        AddEvent(event);        
    }
}
