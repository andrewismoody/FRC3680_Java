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
            event.Complete = false;
        }
    }

    public void Update() {
        elapsedTime = System.currentTimeMillis() - startTime;

        boolean notFinished = false;

        // process triggers
        eventLoop: for (AutoEvent event : Events) {
            if (!event.Complete) {
                notFinished = true;
                switch (event.eventType) {
                    case Time:
                        if (elapsedTime > event.Milliseconds) {
                            System.out.printf("Auto Event %s triggered at %d", event.label, event.Milliseconds);

                            if (event.BoolEvent != null)
                                event.BoolEvent.accept(event.BoolValue);
                            if (event.DoubleEvent != null)
                                event.DoubleEvent.accept(event.DoubleValue);
                            if (event.AutoEvent != null) {
                                autoController.AddSequence(event.AutoEvent);
                            }
                            event.Complete = true;
                            startTime = System.currentTimeMillis();
                        }
                        break;
                    case Position:
                        if (isNearby(controller.GetPosition(), event.Position, 0.5, 1.0)) {
                            System.out.printf("Auto Event %s triggered at %s", event.label, event.Position);

                            if (event.BoolEvent != null)
                                event.BoolEvent.accept(event.BoolValue);
                            if (event.DoubleEvent != null)
                                event.DoubleEvent.accept(event.DoubleValue);
                            if (event.AutoEvent != null) {
                                autoController.AddSequence(event.AutoEvent);
                            }
                            event.Complete = true;
                            startTime = System.currentTimeMillis();
                        }
                        break;
                    case Auto:
                        if (event.AutoEvent != null && event.AutoEvent.IsFinished()) {
                            event.Complete = true;
                            startTime = System.currentTimeMillis();
                        }
                }

                // only break out of the loop if this isn't a parallel event
                if (!event.Parallel)
                    break eventLoop;
            }
        }

        finished = !notFinished;

        controller.ProcessDrive();
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
