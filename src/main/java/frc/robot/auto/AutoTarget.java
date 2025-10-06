package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.modules.ModuleState;

public class AutoTarget {
    public double Distance = 0.0;
    public boolean HasDistance = false;

    public ModuleState State = ModuleState.Off;
    public boolean HasState = false;

    public Translation3d Position = null;
    public boolean HasPosition = false;

    public Rotation2d Orientation = null;
    public boolean HasOrientation = false;

    public Translation3d LookAt = null;
    public boolean HasLookAt = false;

    public AutoTarget(double distance) {
        Distance = distance;
        HasDistance = true;
    }

    public AutoTarget(ModuleState state) {
        State = state;
        HasState = true;
    }

    public AutoTarget(Translation3d translation, boolean isLookat) {
        if (isLookat) {
            LookAt = translation;
            HasLookAt = true;
        } else {
            Position = translation;
            HasPosition = true;
        }
    }

    public AutoTarget(Rotation2d orientation) {
        Orientation = orientation;
        HasOrientation = true;
    }

    public AutoTarget(Translation3d position, Translation3d lookAt) {
        Position = position;
        HasPosition = true;
        LookAt = lookAt;
        HasLookAt = true;
    }

    public AutoTarget(Translation3d position, Rotation2d orientation) {
        Position = position;
        HasPosition = true;
        Orientation = orientation;
        HasOrientation = true;
    }
}
