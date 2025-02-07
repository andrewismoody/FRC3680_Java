package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.auto.AutoEvent.EventType;

public class SequenceShoot extends AutoSequence {
    public SequenceShoot(String Label, ModuleController modules, AutoController MyController) {
        super(Label, modules, MyController);

        // adds an event to start the ejector module
        AutoEventTime event = new AutoEventTime("Start Shooter", false, 0, EventType.Boolean, autoController);
        event.boolEvent = modules.GetModule("ejector")::ProcessState;
        event.boolValue = true;
        AddEvent(event);

        // adds an event to stop the ejector module
        event = new AutoEventTime("Stop Shooter", false, 2000, EventType.Boolean, autoController);
        event.boolEvent = modules.GetModule("ejector")::ProcessState;
        event.boolValue = false;
        AddEvent(event);
    }
}
