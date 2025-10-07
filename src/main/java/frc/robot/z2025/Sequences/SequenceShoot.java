package frc.robot.z2025.Sequences;

import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoEvent.EventType;
import frc.robot.modules.RobotModule;

// SequenceShoot is an autonomous sequence that demonstrates controlling a single module with timed events.
public class SequenceShoot extends AutoSequence {
    public SequenceShoot(String Label, AutoController MyController) {
        super(Label, MyController);
        var modules = MyController.GetModuleController();

        RobotModule ejector = modules.GetModule("ejector");
        if (ejector != null) {
            // adds an event to start the ejector module
            AutoEventTime event = new AutoEventTime("Start Shooter", false, 0, EventType.Boolean, MyController);
            event.SetBoolEvent(true, ejector::ProcessState);
            AddEvent(event);

            // adds an event to stop the ejector module
            event = new AutoEventTime("Stop Shooter", false, 2000, EventType.Boolean, MyController);
            event.SetBoolEvent(false, ejector::ProcessState);
            AddEvent(event);
        }
    }
}
