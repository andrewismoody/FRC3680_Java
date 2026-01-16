package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.misc.GameController;
import frc.robot.modules.ModuleController;

// AutoController manages all AutoSequences, allowing for their addition, removal, initialization, updating, and shutdown.
public class AutoController {
    HashMap<String, AutoSequence> sequences = new HashMap<String, AutoSequence>();

    // NEW: season-definition vocabularies loaded from JSON
    private ArrayList<String> groups = new ArrayList<>();
    private ArrayList<String> locations = new ArrayList<>();
    private ArrayList<String> positions = new ArrayList<>();
    private ArrayList<String> actions = new ArrayList<>();

    // OPTIONAL: fast lookup (label -> index). Keep minimal, but useful for other systems.
    private HashMap<String, Integer> groupIndex = new HashMap<>();
    private HashMap<String, Integer> locationIndex = new HashMap<>();
    private HashMap<String, Integer> positionIndex = new HashMap<>();
    private HashMap<String, Integer> actionIndex = new HashMap<>();

    // NEW: entire parsed season definition for downstream use
    private AutoSeasonDefinition seasonDefinition;

    String label = "unset";
    GameController gameController;
    ModuleController moduleController;
    boolean interrupt = false;

    public AutoController(String Label, ModuleController moduleController) {
        label = Label;
        this.moduleController = moduleController;
    }

    public ModuleController GetModuleController() {
        return moduleController;
    }

    public String GetLabel() {
        return label;
    }

    public void AddSequence(AutoSequence sequence) {
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

    public void Initialize(GameController gameController) {
        this.gameController = gameController;

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

    // NEW: called by parser after reading season JSON
    public void SetDefinitions(
            ArrayList<String> groups,
            ArrayList<String> locations,
            ArrayList<String> positions,
            ArrayList<String> actions) {

        this.groups = (groups != null) ? groups : new ArrayList<>();
        this.locations = (locations != null) ? locations : new ArrayList<>();
        this.positions = (positions != null) ? positions : new ArrayList<>();
        this.actions = (actions != null) ? actions : new ArrayList<>();

        groupIndex.clear();
        locationIndex.clear();
        positionIndex.clear();
        actionIndex.clear();

        for (int i = 0; i < this.groups.size(); i++) groupIndex.put(this.groups.get(i), i);
        for (int i = 0; i < this.locations.size(); i++) locationIndex.put(this.locations.get(i), i);
        for (int i = 0; i < this.positions.size(); i++) positionIndex.put(this.positions.get(i), i);
        for (int i = 0; i < this.actions.size(); i++) actionIndex.put(this.actions.get(i), i);
    }

    // NEW: getters for other classes (read-only usage expected)
    public ArrayList<String> GetGroups() { return groups; }
    public ArrayList<String> GetLocations() { return locations; }
    public ArrayList<String> GetPositions() { return positions; }
    public ArrayList<String> GetActions() { return actions; }

    public Integer GetGroupIndex(String label) { return groupIndex.get(label); }
    public Integer GetLocationIndex(String label) { return locationIndex.get(label); }
    public Integer GetPositionIndex(String label) { return positionIndex.get(label); }
    public Integer GetActionIndex(String label) { return actionIndex.get(label); }

    public AutoSeasonDefinition GetSeasonDefinition() {
        return seasonDefinition;
    }

    public void SetSeasonDefinition(AutoSeasonDefinition def) {
        this.seasonDefinition = def;
    }

    // NEW: list available sequences (for dashboard/chooser)
    public ArrayList<String> GetSequenceLabels() {
        return new ArrayList<>(sequences.keySet());
    }

    // NEW: keep only the selected sequence loaded and ready to run
    public boolean SelectSequence(String sequenceLabel) {
        if (sequenceLabel == null) return false;

        AutoSequence selected = sequences.get(sequenceLabel);
        if (selected == null) return false;

        // shut down and remove all non-selected sequences
        ArrayList<AutoSequence> toRemove = new ArrayList<>();
        for (AutoSequence seq : sequences.values()) {
            if (seq != selected) toRemove.add(seq);
        }
        for (AutoSequence seq : toRemove) RemoveSequence(seq);

        return true;
    }

    // NEW: pick a deterministic default (first inserted is not guaranteed by HashMap,
    // so we choose lexicographically smallest label to keep it stable)
    public String GetDefaultSequenceLabel() {
        String best = null;
        for (String k : sequences.keySet()) {
            if (best == null || k.compareTo(best) < 0) best = k;
        }
        return best;
    }
}
