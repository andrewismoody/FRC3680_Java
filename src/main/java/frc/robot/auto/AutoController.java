package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;

public class AutoController {
    HashMap<String, AutoSequence> sequences = new HashMap<String, AutoSequence>();

    String label = "unset";

    public AutoController(String Label) {
        label = Label;
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

    public void Initialize() {
        for (AutoSequence sequence : sequences.values()) {
            sequence.Initialize();
        }
    }

    public void Update() {
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
