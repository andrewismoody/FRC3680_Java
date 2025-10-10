package frc.robot.modules;

import java.util.ArrayList;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.action.Group;
import frc.robot.auto.AutoTarget;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.encoder.Encoder;
import frc.robot.switches.Switch;

public class SingleMotorModule implements RobotModule {
    String moduleID;
    MotorController driveMotor;
    double driveSpeed;
    boolean invert;
    ModuleController controller;
    PIDController pidController;

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

    double previousTargetDistance = 0.0;
    double encoderMultiplier = 1.0;
    double reverseMultiplier = 1.0;

    private int settleCyclesRequired = 3; // tune as needed
    private int settleCount = 0;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    ActionPose targetPose;
    ArrayList<Consumer<Boolean>> buttonMappedTargets = new ArrayList<Consumer<Boolean>>();
    ArrayList<Consumer<Boolean>> buttonMappedPoses = new ArrayList<Consumer<Boolean>>();

    NetworkTable myTable;
    // Cached NT entries
    private NetworkTableEntry invertEntry;
    private NetworkTableEntry encoderMultiplierEntry;
    private NetworkTableEntry pidSetpointsEntry;
    private NetworkTableEntry targetPoseEntry;
    private NetworkTableEntry settleCountEntry;
    private NetworkTableEntry lowerLimitEntry;
    private NetworkTableEntry targetDistanceEntry;
    private NetworkTableEntry driveDistanceEntry;
    private NetworkTableEntry rotationCountEntry;
    private NetworkTableEntry currentDriveSpeedEntry;

    public SingleMotorModule(String ModuleID, MotorController DriveMotor, double DriveSpeed, boolean Invert, Switch UpperLimit, Switch LowerLimit, Encoder Enc, double EncoderMultiplier, double ReverseMultiplier) {
        moduleID = ModuleID;
        driveMotor = DriveMotor;
        driveSpeed = DriveSpeed;
        invert = Invert;

        var kp = 5; // kp = 20% over max motor capability
        var ki = 0; //kp * 0.10; // ki = 10% of kp
        var kd = 0; //ki * 3; // kd = 3 times ki
        pidController = new PIDController(kp, ki, kd);

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

        // instantiate entries
        invertEntry = myTable.getEntry("invert");
        encoderMultiplierEntry = myTable.getEntry("encoderMultiplier");
        pidSetpointsEntry = myTable.getEntry("pidSetpoints");
        targetPoseEntry = myTable.getEntry("targetPose");
        settleCountEntry = myTable.getEntry("settleCount");
        lowerLimitEntry = myTable.getEntry("lowerLimit");
        targetDistanceEntry = myTable.getEntry("targetDistance");
        driveDistanceEntry = myTable.getEntry("driveDistance");
        rotationCountEntry = myTable.getEntry("rotationCount");
        currentDriveSpeedEntry = myTable.getEntry("currentDriveSpeed");

        // set initial values
        invertEntry.setBoolean(invert);
        encoderMultiplierEntry.setDouble(encoderMultiplier);
        pidSetpointsEntry.setString(String.format("P: %f I: %f D: %f", pidController.getP(), pidController.getI(), pidController.getD()));

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

    public void SetTargetActionPose(ActionPose actionPose) {
        SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
    }    

    public void SetTargetActionPose(Group group, int location, int locationIndex, int position, Action action) {
        var targetPose = GetActionPose(group, location, locationIndex, position, action);
        if (targetPose != null) {
            targetPoseEntry.setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));

            this.targetPose = targetPose;

            // reset settle counter on new target
            settleCount = 0;
            settleCountEntry.setNumber(settleCount);
        }
    }
    
    public ActionPose GetTarget() {
        return targetPose;
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
            lowerLimitEntry.setString("hit");

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

    public void EvaluateTargetPose() {
        // TODO: detect button input and bypass - how would this ever get set to non-zero at this point?
        if (targetPose != null) { //} && currentDriveSpeed != 0.0) {
            double angleTolerance = 0.00001; // 0.00001;
            var target = targetPose.target;
    
            // we have a target and we're not manually applying a value, try to get to it.
            // the x axis of the position of the pose is the rotation count (distance along the motor axis)
            var targetRotation = target.Distance;
            var targetDistance = Math.abs(targetRotation - rotationCount);
            targetDistanceEntry.setDouble(targetDistance);
            previousTargetDistance = targetDistance;

            var shouldMove = (Math.abs(targetDistance) > angleTolerance);
            if (!shouldMove) {
                // increment global settle counter; do not abandon until threshold reached
                if (settleCount < settleCyclesRequired) {
                    settleCount++;
                    settleCountEntry.setNumber(settleCount);
                }

                if (settleCount >= settleCyclesRequired) {
                    AbandonTarget();
                }
            } else {
                currentDriveSpeed = pidController.calculate(rotationCount, targetRotation);
                currentDriveSpeed = controller.ApplyModifiers(invert ? -currentDriveSpeed : currentDriveSpeed);
            }

            var driveDistance = Math.abs(rotationCount - previousRotationCount);
            driveDistanceEntry.setDouble(driveDistance);
        }
    }
    
    @Override
    public void ProcessState(boolean isAuto) {
        if (enc != null) {
            if (enc.isAbsolute())
                setRotationFromAbsolute();
            else
                rotationCount = enc.getRawValue();
            rotationCountEntry.setDouble(rotationCount);
        }

        EvaluateTargetPose();

        currentDriveSpeedEntry.setDouble(currentDriveSpeed);
        previousDriveSpeed = currentDriveSpeed;

        if ((currentDriveSpeed > 0 && (upperLimit == null || !upperLimit.GetState())) ||
            (currentDriveSpeed < 0 && (lowerLimit == null || !lowerLimit.GetState()))) {
                driveMotor.set(currentDriveSpeed);
        } else {
            if (debug  && currentDriveSpeed != 0.0) {
                System.out.printf("%s: limit reached, not driving motor\n", moduleID);
            }
            driveMotor.set(0);

            // prevent AwaitTarget deadlock if motion is blocked by limits
            AbandonTarget();
        }

        previousRotationCount = rotationCount;

        currentDriveSpeed = 0.0;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public String GetModuleID() {
        return moduleID;
    }

    // AddButtonMappedPose adds a position to the array and returns a reference to the function
    // always make a new instance, don't reuse button mappings as they will stomp on each other
    public Consumer<Boolean> AddButtonMappedPose(ActionPose pose) {        
        var setTarget = new Consumer<Boolean>() {
            boolean wasPressed = false;

            @Override
            public void accept(Boolean pressed) {
                if (pressed && !wasPressed) {
                    SetTargetActionPose(pose);
                }
                wasPressed = pressed;
            }
        };
        buttonMappedPoses.add(setTarget);

        return setTarget;
    }

    // GetButtonMappedPose finds the function by index and returns it. Returns null if not found.
    // always make a new instance, don't reuse button mappings as they will stomp on each other
    public Consumer<Boolean> GetButtonMappedPose(int index) {
        if (index >= 0 && index < buttonMappedPoses.size()) {
            return buttonMappedPoses.get(index);
        }

        return null;
    }

    // AddButtonMappedTarget adds a position to the array and returns a reference to the function
    // always make a new instance, don't reuse button mappings as they will stomp on each other
    public Consumer<Boolean> AddButtonMappedTarget(double position) {
        var setTarget = new Consumer<Boolean>() {
            boolean wasPressed = false;

            @Override
            public void accept(Boolean pressed) {
                // multiple mapped consumers defeat this logic as one is pressed and one is not.
                if (pressed && !this.wasPressed) {
                    targetPose = new ActionPose(Group.Any, -1, -1, -1, Action.Any, new AutoTarget(position));
                }
                this.wasPressed = pressed;
            }
        };
        buttonMappedTargets.add(setTarget);

        return setTarget;
    }

    // GetButtonMappedTarget finds the function by index and returns it. Returns null if not found.
    public Consumer<Boolean> GetButtonMappedTarget(int index) {
        if (index >= 0 && index < buttonMappedTargets.size()) {
            return buttonMappedTargets.get(index);
        }

        return null;
    }

    public void SetNoPose(boolean isPressed) {
        if (isPressed) {
            AbandonTarget();
        }
    }

    public void AbandonTarget() {
        targetPose = null;
        targetPoseEntry.setString("none");
    }

    public Pose3d GetPosition() {
        if (enc != null)
            return new Pose3d(new Translation3d(rotationCount, 0, 0), new Rotation3d());
        
        return new Pose3d();
    }
}
