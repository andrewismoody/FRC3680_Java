package frc.robot.z2025;

import java.util.Hashtable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoController;
import frc.robot.misc.GameController;
import frc.robot.modules.ModuleController;
import frc.robot.z2025.Sequences.SequenceControllerMoveToReef;
import frc.robot.z2025.Sequences.SequenceControllerScoreReloadScore;
import frc.robot.z2025.Sequences.SequenceControllerStartScoreReload;
import frc.robot.z2025.Sequences.SequenceRotateScoreReturn;

public class AutoModes {
    public static Hashtable<String, AutoController> Initialize(Hashtable<String, AutoController> AutoModes, GameController m_controller, ModuleController modules) {
        // TODO: need to move this definition to preferences and initialize in automodes rather than hard coding

        AutoController rotateScoreReturn = new AutoController("RotateScoreReturn", m_controller, modules);
        rotateScoreReturn.AddSequence(new SequenceRotateScoreReturn(rotateScoreReturn.GetLabel(), modules, rotateScoreReturn));
        AutoModes.put(rotateScoreReturn.GetLabel(), rotateScoreReturn);

        AutoController rotateWaitReturn = new AutoController("RotateWaitReturn", m_controller, modules);
        rotateWaitReturn.AddSequence(new SequenceRotateScoreReturn(rotateWaitReturn.GetLabel(), modules, rotateWaitReturn));
        AutoModes.put(rotateWaitReturn.GetLabel(), rotateWaitReturn);

        AutoController controllerScoreReloadScore = new AutoController("ControllerScoreReloadScore", m_controller, modules);
        controllerScoreReloadScore.AddSequence(new SequenceControllerScoreReloadScore(controllerScoreReloadScore.GetLabel(), modules, controllerScoreReloadScore));
        AutoModes.put(controllerScoreReloadScore.GetLabel(), controllerScoreReloadScore);

        AutoController controllerMoveToReef = new AutoController("controllerMoveToReef", m_controller, modules);
        controllerMoveToReef.AddSequence(new SequenceControllerMoveToReef(controllerMoveToReef.GetLabel(), modules, controllerMoveToReef));
        AutoModes.put(controllerMoveToReef.GetLabel(), controllerMoveToReef);

        AutoController controllerStartScoreReload = new AutoController("controllerStartScoreReload", m_controller, modules);
        controllerStartScoreReload.AddSequence(new SequenceControllerStartScoreReload(controllerStartScoreReload.GetLabel(), modules, controllerStartScoreReload));
        AutoModes.put(controllerStartScoreReload.GetLabel(), controllerStartScoreReload);

        SmartDashboard.putStringArray("Auto List", new String[] {});
        SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));

        return AutoModes;
    }

    public static AutoController GetDefault(Hashtable<String, AutoController> AutoModes) {
        return AutoModes.get("controllerStartScoreReload");
    }
}
