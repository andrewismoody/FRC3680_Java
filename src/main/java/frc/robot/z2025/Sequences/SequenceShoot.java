package frc.robot.z2025.Sequences;

import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoEvent.EventType;
import frc.robot.modules.RobotModule;

// SequenceShoot is an autonomous sequence that demonstrates controlling a single module with timed events.
public class SequenceShoot extends AutoSequence {
    private final AutoController autoController;
    private boolean initialized = false;
  
      public SequenceShoot(String Label, AutoController ac) {
        super(Label, ac);
        autoController = ac;
    }
  
    @Override
    public void Initialize() {
        super.Initialize();

        if (initialized) {
            System.out.printf("Sequence %s already initialized; skipping duplicate init\n", GetLabel());
            return;
        }
        initialized = true;

        var modules = autoController.GetModuleController();

        RobotModule ejector = modules.GetModule("ejector");
        if (ejector != null) {
            // adds an event to start the ejector module
            AutoEventTime event = new AutoEventTime("Start Shooter", false, 0, EventType.Boolean, autoController);
            event.SetBoolEvent(true, ejector::ProcessState);
            AddEvent(event);

            // adds an event to stop the ejector module
            event = new AutoEventTime("Stop Shooter", false, 2000, EventType.Boolean, autoController);
            event.SetBoolEvent(false, ejector::ProcessState);
            AddEvent(event);
        }
    }
}
