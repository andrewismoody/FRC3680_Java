package frc.robot.auto;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.ModuleController;

public class AutoController implements Auto {

    long startTime = 0;
    long elapsedTime = 0;

    ModuleController controller;

    ArrayList<AutoEvent> Events = new ArrayList<AutoEvent>();

    public AutoController(ModuleController Controller) {
        controller = Controller;
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

        // process triggers
        eventLoop: for (AutoEvent event : Events) {
            if (!event.Complete) {
                switch (event.eventType) {
                    case Time:
                        if (elapsedTime > event.Milliseconds) {
                            System.out.printf("Auto Event %s triggered at %d", event.label, event.Milliseconds);

                            if (event.BoolEvent != null)
                                event.BoolEvent.accept(event.BoolValue);
                            if (event.DoubleEvent != null)
                                event.DoubleEvent.accept(event.DoubleValue);
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
                            event.Complete = true;
                            startTime = System.currentTimeMillis();
                        }
                        break;
                }

                break eventLoop;
            }
        }

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
