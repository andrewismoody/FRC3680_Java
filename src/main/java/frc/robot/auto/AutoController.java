package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.misc.GameController;
import frc.robot.modules.ModuleController;

//TODO: Set module controller here instead of inside events and reference to simplify construction

// AutoController manages all AutoSequences, allowing for their addition, removal, initialization, updating, and shutdown.
public class AutoController {
    HashMap<String, AutoSequence> sequences = new HashMap<String, AutoSequence>();

    String label = "unset";
    GameController gameController;
    ModuleController moduleController;
    boolean interrupt = false;

    public AutoController(String Label, GameController gameController, ModuleController moduleController) {
        label = Label;
        this.gameController = gameController;
        this.moduleController = moduleController;
    }

    public ModuleController GetModuleController() {
        return moduleController;
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
        if (!DriverStation.isAutonomous() && gameController != null) {
            interrupt = gameController.getAnyButton();
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
