package frc.robot.z2025;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoController;
import frc.robot.misc.GameController;
import frc.robot.modules.ModuleController;
import frc.robot.z2025.Sequences.Score3and4;
import frc.robot.z2025.Sequences.Score8and4;

public class AutoModes {
    public static Hashtable<String, AutoController> Initialize(Hashtable<String, AutoController> AutoModes, GameController m_controller, ModuleController modules) {
        // TODO: need to move this definition to preferences and initialize in automodes rather than hard coding

        AutoModes.clear();

        AutoController score3and4 = new AutoController("Score3and4", m_controller, modules);
        score3and4.AddSequence(new Score3and4(score3and4.GetLabel(), score3and4));
        AutoModes.put(score3and4.GetLabel(), score3and4);

        AutoController score8and4 = new AutoController("Score8and4", m_controller, modules);
        score8and4.AddSequence(new Score8and4(score8and4.GetLabel(), score8and4));
        AutoModes.put(score8and4.GetLabel(), score8and4);

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
