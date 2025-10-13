package frc.robot.auto;

import java.util.ArrayList;

import frc.robot.modules.ModuleController;

public class AutoSequence {

    long startTime = 0;
    long elapsedTime = 0;
    String label = "unset";
    boolean finished = false;

    ModuleController controller;
    AutoController autoController;

    ArrayList<AutoEvent> Events = new ArrayList<AutoEvent>();

    boolean interrupt = false;

    public AutoSequence(String Label, AutoController MyController) {
        label = Label;
        autoController = MyController;
        controller = autoController.GetModuleController();
    }

    public String GetLabel() {
        return label;
    }

    public boolean IsFinished() {
        return finished;
    }

    public void AddEvent(AutoEvent event) {
        AutoEvent newEvent;

        switch (event.GetTriggerType()) {
            case Time:
                newEvent = setupTimeEvent((AutoEventTime) event, new AutoEventTime(event.GetLabel(), event.IsParallel(),
                        ((AutoEventTime) event).milliseconds, event.GetEventType(), autoController));
                break;
            case Position:
                newEvent = setupPositionEvent((AutoEventPosition) event, new AutoEventPosition(event.GetLabel(), event.IsParallel(),
                        ((AutoEventPosition) event).target, event.GetEventType(), autoController, controller.GetDriveModule()));
                break;
            case Target:
                newEvent = setupTargetEvent((AutoEventTarget) event, new AutoEventTarget(event.GetLabel(), event.IsParallel(),
                        ((AutoEventTarget) event).target, event.GetEventType(), autoController));
                break;
            case Auto:
                newEvent = new AutoEventAuto(event.GetLabel(), event.IsParallel(), ((AutoEventAuto) event).autoEvent, event.GetEventType());
                break;
            default:
                newEvent = new AutoEventTime(event.GetLabel(), event.IsParallel(), 0, event.GetEventType(), autoController);
                System.out.printf("AutoSequence %s unknown event type %s\n", label, event.GetTriggerType().toString());
                return;
        }

        Events.add(newEvent);
    }

    AutoEventTime setupTimeEvent(AutoEventTime oldEvent, AutoEventTime newEvent) {
        switch (oldEvent.GetEventType()) {
            case Auto:
                newEvent.SetAutoEvent(oldEvent.autoEvent);
                break;
            case Void:
                newEvent.SetVoidEvent(oldEvent.voidEvent);
                break;
            case Boolean:
                newEvent.SetBoolEvent(oldEvent.boolValue, oldEvent.boolEvent);
                break;
            case Double:
                newEvent.SetDoubleEvent(oldEvent.doubleValue, oldEvent.doubleEvent);
                break;
            default:
                // do nothing
                break;
        }

        return newEvent;
    }

    AutoEventPosition setupPositionEvent(AutoEventPosition oldEvent, AutoEventPosition newEvent) {
        switch (oldEvent.GetEventType()) {
            case Auto:
                newEvent.SetAutoEvent(oldEvent.autoEvent);
                break;
            case Void:
                newEvent.SetVoidEvent(oldEvent.voidEvent);
                break;
            case Boolean:
                newEvent.SetBoolEvent(oldEvent.boolValue, oldEvent.boolEvent);
                break;
            case Double:
                newEvent.SetDoubleEvent(oldEvent.doubleValue, oldEvent.doubleEvent);
                break;
            default:
                // do nothing
                break;
        }

        return newEvent;
    }

    AutoEventTarget setupTargetEvent(AutoEventTarget oldEvent, AutoEventTarget newEvent) {
        if (oldEvent.targetModule != null)
            newEvent.targetModule = oldEvent.targetModule;

        return newEvent;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public AutoSequence BeginWith(AutoEvent event) {
        AddEvent(event);
        return this;
    }

    public AutoSequence Then(AutoEvent event) {
        AddEvent(event);
        return this;
    }

    public void Initialize() {
        System.out.printf("AutoSequence %s initializing\n", label);

        startTime = System.currentTimeMillis();
        for (AutoEvent event : Events) {
            event.SetComplete(false);
        }

        interrupt = false;
        finished = false;
    }

    public void Update() {
        if (interrupt)
            return;

        elapsedTime = System.currentTimeMillis() - startTime;

        boolean notFinished = false;

        // TODO figure out why there's a pause between each event
        // process triggers
        eventLoop: for (AutoEvent event : Events) {
            if (interrupt)
                return;

            if (!event.IsComplete()) {
                notFinished = true;
                if (!event.HasStarted()) {
                    System.out.printf("AutoSequence %s starting event %s of type %s\n", label, event.GetLabel(),
                    event.GetTriggerType().toString());
                }

                switch (event.GetTriggerType()) {
                    case Time:
                        AutoEventTime timeEvent = (AutoEventTime) event;
                        if (elapsedTime % 1000 == 0) {
                            // print every second
                            System.out.printf("AutoSequence %s waiting on time event %s: elapsed %d ms of %d ms\n",
                                    label, timeEvent.GetLabel(), elapsedTime, timeEvent.milliseconds);
                        }

                        if (elapsedTime < timeEvent.milliseconds) {
                            timeEvent.Run();
                        } else {
                            timeEvent.SetComplete(true);

                            startTime = System.currentTimeMillis();
                        }
                        break;
                    case Target:
                        AutoEventTarget targetEvent = (AutoEventTarget) event;

                        // AwaitTarget should only complete when the target is null - which occurs inside the Run function
                        switch (targetEvent.GetEventType()) {
                            case AwaitTarget:
                                targetEvent.Run();
                                if (targetEvent.IsComplete())
                                    startTime = System.currentTimeMillis();
                                break;
                            case SetTarget:
                                targetEvent.Run();
                                startTime = System.currentTimeMillis();
                                break;
                            default:
                                // do nothing
                                break;
                        }
                        break;
                    case Position:
                        AutoEventPosition positionEvent = (AutoEventPosition) event;

                        positionEvent.Run();
                        if (positionEvent.IsComplete())
                            startTime = System.currentTimeMillis();
                        break;
                    case Auto:
                        AutoEventAuto autoEvent = (AutoEventAuto) event;
                        // this is an infinite loop - isFinished pingpongs between this class and AutoEventAuto
                        if (autoEvent.IsFinished()) {
                            autoEvent.Run();
                            startTime = System.currentTimeMillis();
                        }
                        break;
                }

                // only break out of the loop if this isn't a parallel event - otherwise, we move to the next event and kick it off
                if (!event.IsParallel())
                    break eventLoop;
            }
        }

        if (!notFinished)
            System.out.printf("AutoSequence %s all events complete\n", label);

        finished = !notFinished;
    }

    public void Shutdown() {
        interrupt = true;

        // stop evaluating and mark complete
        finished = true;

        // clear any active module targets to unblock AwaitTarget events
        if (controller != null) {
            controller.AbandonAllTargets();
        }
    }
}
