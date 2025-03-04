package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.action.Group;
import frc.robot.action.Location;
import frc.robot.action.Position;


public class SingleActuatorModule implements RobotModule {

    Relay relay;
    String moduleID;
    Value currentValue = Value.kOff;
    Boolean invert;
    ModuleController controller;

    public SingleActuatorModule(String ModuleID, Relay Relay, Boolean Invert) {
        moduleID = ModuleID;
        relay = Relay;
        invert = Invert;
    }

    @Override
    public void Initialize() {

    }

    @Override
    public void ProcessState(boolean isAuto) {
        relay.set(currentValue);
    }

    @Override
    public void ApplyValue(boolean value) {
        if (value) {
            if (invert) {
                currentValue = Value.kOff;
            } else {
                currentValue = Value.kOn;
            }
        }
    }

    @Override
    public void ApplyInverse(boolean value) {
        if (value) {
            if (invert) {
                currentValue = Value.kOn;
            } else {
                currentValue = Value.kOff;
            }
        }
    }

    @Override
    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    @Override
    public String GetModuleID() {
        return moduleID;
    }

    @Override
    public void SetTargetActionPose(Group Group, Location Location, int LocationIndex, Position Position,
            Action Action) {
        // TODO do this
        throw new UnsupportedOperationException("Unimplemented method 'SetTargetActionPose'");
    }

    @Override
    public void SetTargetActionPose(ActionPose ActionPose) {
        SetTargetActionPose(ActionPose.group, ActionPose.location, ActionPose.locationIndex, ActionPose.position, ActionPose.action);
    }

    @Override
    public Pose3d GetPosition() {
        return new Pose3d(new Translation3d(currentValue == Value.kOff ? 0 : currentValue == Value.kForward ? 1 : -1, 0, 0), new Rotation3d());
    }

    @Override
    public void AddActionPose(ActionPose NewPose) {
        // TODO do this
        throw new UnsupportedOperationException("Unimplemented method 'AddActionPose'");
    }

    @Override
    public ActionPose GetActionPose(Group Group, Location Location, int LocationIndex, Position Position,
            Action Action) {
        // TODO do this
        throw new UnsupportedOperationException("Unimplemented method 'GetActionPose'");
    }
    
}
