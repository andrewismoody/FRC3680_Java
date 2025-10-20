package frc.robot.z2024;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoController;
import frc.robot.modules.ModuleController;
import frc.robot.z2024.Sequences.ScoreSpeaker;

public class AutoModes {
    public static Hashtable<String, AutoController> Initialize(Hashtable<String, AutoController> AutoModes, ModuleController modules) {
        // TODO: need to move this definition to preferences and initialize in automodes rather than hard coding

        AutoModes.clear();

        AutoController scoreSpeaker = new AutoController("ScoreSpeaker",modules);
        scoreSpeaker.AddSequence(new ScoreSpeaker(scoreSpeaker.GetLabel(), scoreSpeaker));
        AutoModes.put(scoreSpeaker.GetLabel(), scoreSpeaker);

        SmartDashboard.putStringArray("Auto List", new String[] {});
        SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));

        return AutoModes;
    }

    public static AutoController GetDefault(Hashtable<String, AutoController> AutoModes) {
        var mode = AutoModes.get("ScoreSpeaker");
        if (mode != null)
            System.out.printf("Default Auto Mode: %s; mode count: %d\n", mode.GetLabel(), AutoModes.size());
        else
            System.out.printf("Default Auto Mode: None found; mode count: %d\n", AutoModes.size());
        return mode;
    }
}
