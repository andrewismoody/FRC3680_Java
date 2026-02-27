package frc.robot.z2026;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoController;
import frc.robot.modules.ModuleController;
import frc.robot.z2026.Sequences.Score4andReload;

public class AutoModes {
    public static Hashtable<String, AutoController> Initialize(Hashtable<String, AutoController> AutoModes, ModuleController modules) {
        // TODO: need to move this definition to preferences and initialize in automodes rather than hard coding

        AutoModes.clear();

        AutoController score3and4 = new AutoController("Score3and4",modules);
        score3and4.AddSequence(new Score4andReload(score3and4.GetLabel(), score3and4));
        AutoModes.put(score3and4.GetLabel(), score3and4);

        SmartDashboard.putStringArray("Auto List", new String[] {});
        SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));

        return AutoModes;
    }

    public static AutoController GetDefault(Hashtable<String, AutoController> AutoModes) {
        var mode = AutoModes.get("Score3and4");
        if (mode != null)
            System.out.printf("Default Auto Mode: %s; mode count: %d\n", mode.GetLabel(), AutoModes.size());
        else
            System.out.printf("Default Auto Mode: None found; mode count: %d\n", AutoModes.size());
        return mode;
    }
}
