package frc.robot.z2025.Sequences;

import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEventAuto;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.auto.AutoEvent.EventType;

// SequenceMoveAndShoot is an example of using time events to move the robot and nested sequences.
public class SequenceMoveAndShoot extends AutoSequence {
    public SequenceMoveAndShoot(String Label, AutoController MyController) {
        super(Label, MyController);
        var modules = MyController.GetModuleController();

        // adds an event to start driving the motors immediately
        AutoEventTime event0 = new AutoEventTime("Move Forward", true, 0, EventType.Double, MyController);
        event0.SetDoubleEvent(1.0, modules.GetDriveModule()::ProcessForwardSpeed);
        AddEvent(event0);

        // adds an event to stop driving the motors after some time
        AutoEventTime event2 = new AutoEventTime("Stop Driving", false, 3000, EventType.Double, MyController);
        event2.SetDoubleEvent(0.0, modules.GetDriveModule()::ProcessForwardSpeed);
        AddEvent(event2);

        // adds an event to trigger the nested sequence at a particular time
        AutoSequence ShootSequence = new SequenceShoot("Shoot", MyController);
        AutoEventTime event4 = new AutoEventTime("Start Shoot", false, 2000, EventType.Auto, MyController);
        event4.SetAutoEvent(ShootSequence);
        AddEvent(event4);

        // adds an event to capture when the nested sequence is finished (need to send
        // the same sequence instance here so we can check IsFinished())
        AutoEventAuto event5 = new AutoEventAuto("Finish Shoot", false, ShootSequence, EventType.Auto);
        AddEvent(event5);
     }
}
