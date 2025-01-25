package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.ModuleController;

public class AutoSequence {

    long startTime = 0;
    long elapsedTime = 0;
    String label = "unset";
    boolean finished = false;

    ModuleController controller;
    AutoController autoController;

    ArrayList<AutoEvent> Events = new ArrayList<AutoEvent>();

    public AutoSequence(String Label, ModuleController Controller, AutoController MyController) {
        label = Label;
        controller = Controller;
        autoController = MyController;
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
    }

    public void Update() {
        elapsedTime = System.currentTimeMillis() - startTime;

        boolean notFinished = false;

        // process triggers
        eventLoop: for (AutoEvent event : Events) {
            if (!event.IsComplete()) {
                notFinished = true;
                switch (event.GetTriggerType()) {
                    case Time:
                        AutoEventTime timeEvent = (AutoEventTime) event;
                        if (elapsedTime > timeEvent.milliseconds) {
                            System.out.printf("Auto Event %s triggered at %d", timeEvent.GetLabel(),
                                    timeEvent.GetMilliseconds());

                            timeEvent.Run();
                            startTime = System.currentTimeMillis();
                        }
                        break;
                    case Position:
                        AutoEventPosition positionEvent = (AutoEventPosition) event;
                        if (isNearby(controller.GetPosition(), positionEvent.target, 0.5, 1.0)) {
                            System.out.printf("Auto Event %s triggered at %s", event.GetLabel(), positionEvent.target);

                            positionEvent.Run();
                            startTime = System.currentTimeMillis();
                        }
                        break;
                    case Auto:
                        AutoEventAuto autoEvent = (AutoEventAuto) event;
                        if (autoEvent.IsFinished()) {
                            autoEvent.Run();
                            startTime = System.currentTimeMillis();
                        }
                }

                // only break out of the loop if this isn't a parallel event - otherwise, we move to the next event and kick it off
                if (!event.IsParallel())
                    break eventLoop;
            }
        }

        finished = !notFinished;

        controller.ProcessDrive(true);
    }

    boolean isNearby(Translation3d Position, Translation3d Target, double PositionTolerance, double AngleTolerance) {
        if (Math.abs(Position.getX() - Target.getX()) > PositionTolerance)
            return false;

        if (Math.abs(Position.getY() - Target.getY()) > PositionTolerance)
            return false;

        if (Math.abs(Position.getZ() - Target.getZ()) > AngleTolerance)
            return false;

        return true;
    }

    public void Shutdown() {

    }
}
