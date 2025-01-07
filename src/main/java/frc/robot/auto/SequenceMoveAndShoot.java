package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.auto.AutoEvent.EventType;

public class SequenceMoveAndShoot extends AutoSequence {
    public SequenceMoveAndShoot(String Label, ModuleController modules, AutoController MyController) {
        super(Label, modules, MyController);

        // adds an event to start driving the motors immediately
        AutoEvent event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event.DoubleValue = -1.0;
        event.Milliseconds = 0;
        event.label = "Move Backward";
        event.Parallel = true;
        AddEvent(event);

        // adds an event to start turning the robot immediately - this will run at the same time as the forward driving event
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessRotationAngle;
        event.DoubleValue = 30.0;
        event.Milliseconds = 0;
        event.label = "Rotate 30 Degrees";
        AddEvent(event);

        // adds an event to stop driving the motors after some time
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event.DoubleValue = 0.0;
        event.Milliseconds = 2000;
        event.label = "Stop Driving";
        AddEvent(event);

        // adds an event to stop turning the robot immediately
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.VoidEvent = modules.GetDriveModule()::StopRotation;
        event.Milliseconds = 0;
        event.label = "Stop Turning";
        AddEvent(event);

        // adds an event to trigger the nested sequence at a particular time
        AutoSequence ShootSequence = new SequenceShoot("Shoot", modules, MyController);
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.AutoEvent = ShootSequence;
        event.Milliseconds = 2000;
        event.label = "Start Shoot";
        AddEvent(event);

        // adds an event to capture when the nested sequence is finished (need to send the same sequence instance here so we can check IsFinished())
        event = new AutoEvent();
        event.eventType = EventType.Auto;
        event.AutoEvent = ShootSequence;
        event.label = "Finish Shoot";
        AddEvent(event);

        // adds an event to start driving the motors immediately
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event.DoubleValue = 1.0;
        event.Milliseconds = 0;
        event.label = "Move Forward";
        AddEvent(event);

        // adds an event to stop driving the motors after some time
        event = new AutoEvent();
        event.eventType = EventType.Time;
        event.DoubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
        event.DoubleValue = 0.0;
        event.Milliseconds = 2000;
        event.label = "Stop";
        AddEvent(event);
    }
}
