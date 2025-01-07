package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.auto.AutoEvent.EventType;

public class SequenceShoot extends AutoSequence {
    public SequenceShoot(String Label, ModuleController modules, AutoController MyController) {
        super(Label, modules, MyController);

        // adds an event to start the ejector module
        AutoEvent event = new AutoEvent();
        event.eventType = EventType.Time;
        event.BoolEvent = modules.GetModule("ejector")::ProcessState;
        event.BoolValue = true;
        event.Milliseconds = 0;
        event.label = "Start Shooter";
        AddEvent(event);

        // adds an event to stop the ejector module
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.BoolEvent = modules.GetModule("ejector")::ProcessState;
        event.BoolValue = false;
        event.Milliseconds = 2000;
        event.label = "Stop Shooter";
        AddEvent(event);
    }
}
