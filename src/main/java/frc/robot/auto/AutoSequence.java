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
        Events.add(event);
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public void Initialize() {
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

        // process triggers
        eventLoop: for (AutoEvent event : Events) {
            if (interrupt)
                return;

            if (!event.IsComplete()) {
                notFinished = true;
                switch (event.GetTriggerType()) {
                    case Time:
                        AutoEventTime timeEvent = (AutoEventTime) event;
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
