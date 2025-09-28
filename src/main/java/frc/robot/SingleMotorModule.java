package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.action.*;
import frc.robot.encoder.Encoder;
import frc.robot.switches.Switch;

public class SingleMotorModule implements RobotModule {
    String moduleID;
    MotorController driveMotor;
    double driveSpeed;
    boolean invert;
    ModuleController controller;

    Encoder enc;
    Switch upperLimit;
    Switch lowerLimit;

    double fullRotation = 360.0; // degrees
    double previousEncValue = 0.0;
    double rotationCount = 0.0;
    double previousRotationCount = 0.0;
    double m_floatTolerance = 0.3f;

    public boolean debug = false;
    double previousDriveSpeed;
    double currentDriveSpeed;

    int sampleCount = 0;
    int sampleMin = 20;
    double maxDistance = 0.0;
    double previousTargetDistance = 0.0;
    double encoderMultiplier = 1.0;
    double reverseMultiplier = 1.0;

    Pose3d target;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();

    NetworkTable myTable;

    public SingleMotorModule(String ModuleID, MotorController DriveMotor, double DriveSpeed, boolean Invert, Switch UpperLimit, Switch LowerLimit, Encoder Enc, double EncoderMultiplier, double ReverseMultiplier) {
        moduleID = ModuleID;
        driveMotor = DriveMotor;
        driveSpeed = DriveSpeed;
        invert = Invert;

        upperLimit = UpperLimit;
        lowerLimit = LowerLimit;
        enc = Enc;
        encoderMultiplier = EncoderMultiplier;
        reverseMultiplier = ReverseMultiplier;

        if (enc != null && enc.isAbsolute())
            previousEncValue = getEncValAdj();
    }

    public void Initialize() {
        myTable = NetworkTableInstance.getDefault().getTable(moduleID);

        myTable.getEntry("invert").setBoolean(invert);
        myTable.getEntry("encoderMultiplier").setDouble(encoderMultiplier);

        if (enc != null) {
            enc.setMultiplier(encoderMultiplier);
            enc.setZeroPosition();
        }
    }

    public void AddActionPose(ActionPose newAction) {
        if (GetActionPose(newAction) == null) {
            actionPoses.add(newAction);
        }
    }

    public ActionPose GetActionPose(ActionPose newAction) {
        return GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action);
    }

    public ActionPose GetActionPose(Group group, Location location, int locationIndex, Position position, Action action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || pose.group == Group.Any)
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || pose.location == Location.Any)
                && (pose.position == position || pose.position == Position.Any)
                && (pose.action == action || pose.action == Action.Any)
            ) {
                return pose;
            }
        }

        return null;      
    }

    public void SetTargetActionPose(ActionPose actionPose) {
        SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
    }    

    public void SetTargetActionPose(Group group, Location location, int locationIndex, Position position, Action action) {
        var targetPose = GetActionPose(group, location, locationIndex, position, action);
        if (targetPose != null) {
            myTable.getEntry("targetPose").setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));

            target = targetPose.pose;
        }
    }    

    double getEncValAdj() {
        var encVal = enc.getDistance();
        encVal = encVal < 0 ? fullRotation + encVal : encVal;
        encVal = encVal % fullRotation;
        return encVal;        
    }

    void setRotationFromAbsolute() {
        var encVal = getEncValAdj();
        var delta = encVal - previousEncValue;

        if (lowerLimit != null && lowerLimit.GetState()) {
            myTable.getEntry("lowerLimit").setString("hit");

            System.out.printf("%s: limit hit, resetting rotationCount\n", moduleID);
            rotationCount = 0.0;
        }
        else {
            if (delta > fullRotation * 0.5 && encVal > fullRotation * 0.75)
                // we crossed zero boundary going backwards
                delta -= fullRotation;
            else if (delta < -(fullRotation * 0.5) && encVal < fullRotation * 0.25)
                // we crossed zero boundary going forwards
                delta += fullRotation;

            // filter noisy responses
            if (Math.abs(delta) < fullRotation * 0.5) {
                rotationCount += (invert ? -delta : delta);
            }
            else
                System.out.printf("%s: throwing away errant value %f\n", moduleID, delta);
        }

        previousEncValue = encVal;
    }

    @Override
    public void ApplyInverse(boolean isPressed) {
        if (isPressed) {
            AbandonTarget();
            currentDriveSpeed += controller.ApplyModifiers(invert ? driveSpeed : -driveSpeed) * reverseMultiplier;
        }
    }

    @Override
    public void ApplyValue(boolean isPressed) {
        if (isPressed) {
            currentDriveSpeed += controller.ApplyModifiers(invert ? -driveSpeed : driveSpeed);
            AbandonTarget();
        }
    }
    
    @Override
    public void ProcessState(boolean isAuto) {
        if (enc != null) {
            if (enc.isAbsolute())
                setRotationFromAbsolute();
            else
                rotationCount = enc.getRawValue();
        }

        double angleTolerance = 0.00001; // 0.00001;

        if (target != null && currentDriveSpeed == 0.0) {
            // we have a target and we're not manually applying a value, try to get to it.
            var targetRotation = target.getX();
            var targetDistance = Math.abs(targetRotation - rotationCount);
            myTable.getEntry("targetDistance").setDouble(targetDistance);
            previousTargetDistance = targetDistance;

            var shouldMove = (Math.abs(targetDistance) > angleTolerance);
            if (targetRotation > rotationCount) {
                if (shouldMove) {
                    currentDriveSpeed += controller.ApplyModifiers(invert ? -driveSpeed : driveSpeed);
                }
            } else if (targetRotation < rotationCount) {
                if (shouldMove) {
                    currentDriveSpeed += controller.ApplyModifiers(invert ? driveSpeed : -driveSpeed);
                }
            }

            var driveDistance = Math.abs(rotationCount - previousRotationCount);
            myTable.getEntry("driveDistance").setDouble(driveDistance);
            
            if (sampleCount >= sampleMin && targetDistance < maxDistance) {
                var adjustmentFactor = (targetDistance / maxDistance);
                //System.out.printf("%s achieved sample count; adjusting motorSpeed by factor %f\n", moduleID, adjustmentFactor);
                currentDriveSpeed *= adjustmentFactor;
            }

            sampleCount++;
        }


        myTable.getEntry("currentDriveSpeed").setDouble(currentDriveSpeed);
        previousDriveSpeed = currentDriveSpeed;

        if ((currentDriveSpeed > 0 && (upperLimit == null || !upperLimit.GetState())) ||
            (currentDriveSpeed < 0 && (lowerLimit == null || !lowerLimit.GetState()))) {
                driveMotor.set(currentDriveSpeed);
        } else {
            if (debug  && currentDriveSpeed != 0.0) {
                System.out.printf("%s: limit reached, not driving motor\n", moduleID);
            }
            driveMotor.set(0);
        }

        myTable.getEntry("rotationCount").setDouble(rotationCount);
        previousRotationCount = rotationCount;

        currentDriveSpeed = 0.0;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public String GetModuleID() {
        return moduleID;
    }

    public void SetScoringPoseMiddle(boolean isPressed) {
        if (isPressed) {
            SetTargetActionPose(Group.Score, Location.Any, -1, Position.Middle, Action.Any);
        }
    }
    public void SetScoringPoseLower(boolean isPressed) {
        if (isPressed) {
            SetTargetActionPose(Group.Score, Location.Any, -1, Position.Lower, Action.Any);
        }
    }
    public void SetScoringPoseTrough(boolean isPressed) {
        if (isPressed) {
            SetTargetActionPose(Group.Score, Location.Any, -1, Position.Trough, Action.Any);
        }
    }

    public void SetNoPose(boolean isPressed) {
        if (isPressed) {
            System.out.printf("%s: Setting No Pose\n", moduleID);
            AbandonTarget();
        }
    }


    public void AbandonTarget() {
        if (debug) {
            System.out.printf("%s: Setting No Pose\n", moduleID);
        }
        target = null;
    }

    public Pose3d GetPosition() {
        if (enc != null)
            return new Pose3d(new Translation3d(rotationCount, 0, 0), new Rotation3d());
        
        return new Pose3d();
    }
}
