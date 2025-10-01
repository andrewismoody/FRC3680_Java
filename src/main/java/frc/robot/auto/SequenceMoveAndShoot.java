package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.auto.AutoEvent.EventType;

// SequenceMoveAndShoot is an example of using time events to move the robot and nested sequences.
public class SequenceMoveAndShoot extends AutoSequence {
    public SequenceMoveAndShoot(String Label, ModuleController modules, AutoController MyController) {
        super(Label, modules, MyController);

        // adds an event to start driving the motors immediately
        AutoEventTime event0 = new AutoEventTime("Move Forward", true, 0, EventType.Double, autoController);
        event0.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event0.doubleValue = 1.0;
        AddEvent(event0);

        // adds an event to stop driving the motors after some time
        AutoEventTime event2 = new AutoEventTime("Stop Driving", false, 3000, EventType.Double, autoController);
        event2.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event2.doubleValue = 0.0;
        AddEvent(event2);

        // adds an event to trigger the nested sequence at a particular time
        AutoSequence ShootSequence = new SequenceShoot("Shoot", modules, MyController);
        AutoEventTime event4 = new AutoEventTime("Start Shoot", false, 2000, EventType.Auto, autoController);
        event4.autoEvent = ShootSequence;
        AddEvent(event4);

        // adds an event to capture when the nested sequence is finished (need to send
        // the same sequence instance here so we can check IsFinished())
        AutoEventAuto event5 = new AutoEventAuto("Finish Shoot", false, ShootSequence, EventType.Auto);
        AddEvent(event5);
     }
}
