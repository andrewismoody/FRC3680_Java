package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.GameController;

// AutoController manages all AutoSequences, allowing for their addition, removal, initialization, updating, and shutdown.
public class AutoController {
    HashMap<String, AutoSequence> sequences = new HashMap<String, AutoSequence>();

    String label = "unset";
    GameController m_controller;
    boolean interrupt = false;

    public AutoController(String Label, GameController controller) {
        label = Label;
        m_controller = controller;
    }

    public String GetLabel() {
        return label;
    }

    public void AddSequence(AutoSequence sequence) {
        sequence.Initialize();
        sequences.put(sequence.label, sequence);
    }

    public void RemoveSequence(AutoSequence sequence) {
        sequence.Shutdown();
        sequences.remove(sequence.label);
    }

    public void InterruptAll(boolean value) {
        // value is unused from button mapper

        for (AutoSequence sequence : sequences.values()) {
            sequence.Shutdown();
        }
        sequences.clear();
    }

    public void Initialize() {
        for (AutoSequence sequence : sequences.values()) {
            sequence.Initialize();
        }
    }

    public void Update() {
        if (!DriverStation.isAutonomous() && m_controller != null) {
            interrupt = m_controller.getAnyButton();
            if (interrupt) {
                InterruptAll(true);
                return;
            }
        }

        ArrayList<AutoSequence> Finished = new ArrayList<>();
        for (AutoSequence sequence : sequences.values()) {
            sequence.Update();
            if (sequence.IsFinished()) {
                Finished.add(sequence);
            }
        }

        // remove these after the loop completes because we can't modify an enum while evaluating it
        for (AutoSequence sequence : Finished) {
            RemoveSequence(sequence);
        }
    }

    public void Shutdown() {
        for (AutoSequence sequence : sequences.values()) {
            sequence.Shutdown();
        }
    }
}
