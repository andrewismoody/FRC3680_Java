package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.auto.AutoEvent.EventType;

public class SequenceMoveAndShoot extends AutoSequence {
    public SequenceMoveAndShoot(String Label, ModuleController modules, AutoController MyController) {
        super(Label, modules, MyController);

        // adds an event to start driving the motors immediately
        AutoEventTime event0 = new AutoEventTime("Move Forward", true, 0, EventType.Double, autoController);
        event0.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event0.doubleValue = 1.0;
        AddEvent(event0);

        // adds an event to start turning the robot immediately - this will run at the
        // same time as the forward driving event
        // AutoEventTime event1 = new AutoEventTime("Rotate 30 Degrees", false, 0, EventType.Double, autoController);
        // event1.doubleEvent = modules.GetDriveModule()::ProcessRotationAngle;
        // event1.doubleValue = 30.0;
        // AddEvent(event1);

        // adds an event to stop driving the motors after some time
        AutoEventTime event2 = new AutoEventTime("Stop Driving", false, 3000, EventType.Double, autoController);
        event2.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event2.doubleValue = 0.0;
        AddEvent(event2);

        // adds an event to stop turning the robot immediately
    //     AutoEventTime event3 = new AutoEventTime("Stop Turning", false, 0, EventType.Void, autoController);
    //     event3.voidEvent = modules.GetDriveModule()::StopRotation;
    //     AddEvent(event3);

    //     // adds an event to trigger the nested sequence at a particular time
    //     AutoSequence ShootSequence = new SequenceShoot("Shoot", modules, MyController);
    //     AutoEventTime event4 = new AutoEventTime("Start Shoot", false, 2000, EventType.Auto, autoController);
    //     event4.autoEvent = ShootSequence;
    //     AddEvent(event4);

    //     // adds an event to capture when the nested sequence is finished (need to send
    //     // the same sequence instance here so we can check IsFinished())
    //     AutoEventAuto event5 = new AutoEventAuto("Finish Shoot", false, ShootSequence, EventType.Auto);
    //     AddEvent(event5);

    //     // adds an event to start driving the motors immediately
    //     AutoEventTime event = new AutoEventTime("Move Forward", false, 0, EventType.Double, autoController);
    //     event.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
    //     event.doubleValue = 1.0;
    //     AddEvent(event);

    //     // adds an event to stop driving the motors after some time
    //     event = new AutoEventTime("Stop", false, 2000, EventType.Double, autoController);
    //     event.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
    //     event.doubleValue = 0.0;
    //     AddEvent(event);
     }
}
