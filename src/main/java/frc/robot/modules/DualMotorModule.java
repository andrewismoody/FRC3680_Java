package frc.robot.modules;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry; // added
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.action.Group;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.encoder.Encoder;
import frc.robot.switches.Switch;

public class DualMotorModule implements RobotModule {
    public enum MotorSide {
        Left,
        Right
    }
    
    String moduleID;
    MotorController leftDriveMotor;
    MotorController rightDriveMotor;
    double driveSpeed;
    boolean invertLeft;
    boolean invertRight;
    ModuleController controller;

    public boolean debug = false;
    double previousDriveSpeed;
    double currentDriveSpeed;

    Encoder leftEnc;
    Encoder rightEnc;
    Switch upperLimit;
    Switch lowerLimit;

    double rightRotationCount = 0.0;
    double leftRotationCount = 0.0;
    double fullRotation = 6.28;
    double previousTargetDistance = 0.0;

    double previousleftEncValue = 0.0;
    double previousRightEncValue = 0.0;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    ActionPose targetPose;
    MotorSide primaryMotor = MotorSide.Left;

    private int settleCyclesRequired = 3; // tune as needed
    private int settleCount = 0;

    NetworkTable myTable;
    // Cached entries
    private NetworkTableEntry ntInvertLeft;
    private NetworkTableEntry ntInvertRight;
    private NetworkTableEntry ntTarget;
    private NetworkTableEntry ntTargetDistance;
    private NetworkTableEntry ntSettleCount;
    private NetworkTableEntry ntRotationCount;
    private NetworkTableEntry ntCurrentDriveSpeed;
    private NetworkTableEntry ntDriveDistance;
    private NetworkTableEntry ntTargetPose;

    public DualMotorModule(String ModuleID, MotorController LeftDriveMotor, MotorController RightDriveMotor, double DriveSpeed, boolean InvertLeft, boolean InvertRight, Switch UpperLimit, Switch LowerLimit, Encoder RightEnc, Encoder LeftEnc) {
        moduleID = ModuleID;
        leftDriveMotor = LeftDriveMotor;
        rightDriveMotor = RightDriveMotor;
        driveSpeed = DriveSpeed;
        invertLeft = InvertLeft;
        invertRight = InvertRight;

        leftEnc = LeftEnc;
        rightEnc = RightEnc;
        upperLimit = UpperLimit;
        lowerLimit = LowerLimit;
    }

    public void Initialize() {
        // Initialize table and cache entries
        myTable = NetworkTableInstance.getDefault().getTable(moduleID);
        ntInvertLeft = myTable.getEntry("invertLeft");
        ntInvertRight = myTable.getEntry("invertRight");
        ntTarget = myTable.getEntry("target");
        ntTargetDistance = myTable.getEntry("targetDistance");
        ntSettleCount = myTable.getEntry("settleCount");
        ntRotationCount = myTable.getEntry("rotationCount");
        ntCurrentDriveSpeed = myTable.getEntry("currentDriveSpeed");
        ntDriveDistance = myTable.getEntry("driveDistance");
        ntTargetPose = myTable.getEntry("targetPose");

        // Initial values
        ntInvertLeft.setBoolean(invertLeft);
        ntInvertRight.setBoolean(invertRight);
        ntTarget.setString("none");
        ntSettleCount.setNumber(0);
        ntRotationCount.setDouble(0.0);
        ntCurrentDriveSpeed.setDouble(0.0);
        ntDriveDistance.setDouble(0.0);
    }

    public void AddActionPose(ActionPose newAction) {
        if (GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action) == null) {
            actionPoses.add(newAction);
        }
    }

    public ActionPose GetActionPose(Group group, int location, int locationIndex, int position, Action action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || pose.group == Group.Any)
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || pose.location == -1)
                && (pose.position == position || pose.position == -1)
                && (pose.action == action || pose.action == Action.Any)
            ) {
                return pose;
            }
        }

        return null;      
    }

    public ActionPose GetTarget() {
        return targetPose;
    }

    @Override
    public void ApplyInverse(boolean isPressed) {
        if (isPressed) {
            if (debug)
                System.out.printf("%s: ApplyInverse\n", moduleID);
            currentDriveSpeed += controller.ApplyModifiers(-driveSpeed);
        }
    }

    @Override
    public void ApplyValue(boolean isPressed) {
        if (isPressed) {
            if (debug)
                System.out.printf("%s: ApplyValue\n", moduleID);
            currentDriveSpeed += controller.ApplyModifiers(driveSpeed);
        }  
    }

    public void EvaluateTargetPose(double rotationCount) {
        // TODO: Detect button input and bypass - how would this ever get set to non-zero at this point?
        if (targetPose != null && currentDriveSpeed == 0.0) {
            double angleTolerance = 0.00001; // 0.00001;
            var target = targetPose.target;
    
            // we have a target and we're not manually applying a value, try to get to it.
            // the x axis of the position of the pose is the rotation count (distance along the motor axis)
            var targetRotation = target.Distance;
            var targetDistance = Math.abs(targetRotation - rotationCount);
            ntTargetDistance.setDouble(targetDistance);
            previousTargetDistance = targetDistance;

            var shouldMove = (Math.abs(targetDistance) > angleTolerance);
            if (!shouldMove) {
                // increment global settle counter; do not abandon until threshold reached
                if (settleCount < settleCyclesRequired) {
                    settleCount++;
                    ntSettleCount.setNumber(settleCount);
                }
                
                if (settleCount >= settleCyclesRequired) {
                    AbandonTarget();
                }
            } else {
                if (targetRotation > rotationCount) {
                    currentDriveSpeed += driveSpeed;
                } else if (targetRotation < rotationCount) {
                    currentDriveSpeed -= driveSpeed;
                }
            }
        }
    }
    
    public void ProcessState(boolean isAuto) {
        // TODO: comb through this and make sure it's sane. 
        var enc = primaryMotor == MotorSide.Left ? leftEnc : rightEnc;
        var rotationCount = primaryMotor == MotorSide.Left ? leftRotationCount : rightRotationCount;

        if (enc != null) {
            // check how to do Absolute for this
            // if (enc.isAbsolute())
            //     setRotationFromAbsolute();
            // else
                rotationCount = enc.getRawValue();

            // encoder interaction goes here so we write it out no matter what
            ntRotationCount.setDouble(rotationCount);
        }

        EvaluateTargetPose(rotationCount);
        ntCurrentDriveSpeed.setDouble(currentDriveSpeed);
        
        if ((currentDriveSpeed > 0 && (upperLimit == null || !upperLimit.GetState())) ||
            (currentDriveSpeed < 0 && (lowerLimit == null || !lowerLimit.GetState()))) {
                leftDriveMotor.set(invertLeft ? -currentDriveSpeed : currentDriveSpeed);
                rightDriveMotor.set(invertRight ? -currentDriveSpeed : currentDriveSpeed);
        } else {
            if (debug && currentDriveSpeed != 0.0) {
                System.out.println("limit reached, not driving motor");
            }
            leftDriveMotor.set(0);
            rightDriveMotor.set(0);

            // If target exists but motion is blocked, abandon to unblock AwaitTarget
            if (targetPose != null) {
                AbandonTarget();
            }
        }

        var prevRotationCount = primaryMotor == MotorSide.Left ? leftRotationCount : rightRotationCount;

        if (lowerLimit != null && lowerLimit.GetState()) {
            leftRotationCount = 0.0;
            rightRotationCount = 0.0;
        }
        else {
            if (leftEnc != null)
                leftRotationCount += fullRotation - (previousleftEncValue - leftEnc.getDistance());
            if (rightEnc != null)
                rightRotationCount += fullRotation - (previousRightEncValue - rightEnc.getDistance());
        }

        ntDriveDistance.setDouble(prevRotationCount - (primaryMotor == MotorSide.Left ? leftRotationCount : rightRotationCount));

        if (rightEnc != null) {
            previousRightEncValue = rightEnc.getDistance();
        }

        if (leftEnc != null) {
            previousleftEncValue = leftEnc.getDistance();
        }

        currentDriveSpeed = 0.0;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }   

    public String GetModuleID() {
        return moduleID;
    }

    public void SetTargetActionPose(Group group, int location, int locationIndex, int position, Action action) {
        var targetPose = GetActionPose(group, location, locationIndex, position, action);
        if (targetPose != null) {
            ntTargetPose.setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));

            this.targetPose = targetPose;

            // reset settle counter on new target
            settleCount = 0;
            ntSettleCount.setNumber(settleCount);
        }
    }    

    public void SetTargetActionPose(ActionPose actionPose) {
        SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
    }   

    public Pose3d GetPosition() {
        double leftVal = 0;
        double rightVal = 0;

        if (leftEnc != null)
            leftVal = leftEnc.getDistance();

        if (rightEnc != null)
            rightVal = rightEnc.getDistance();

        return new Pose3d(new Translation3d(leftVal, rightVal, 0), new Rotation3d());
    }

    public void AbandonTarget() {
        targetPose = null;
        ntTarget.setString("none");
    }
}
