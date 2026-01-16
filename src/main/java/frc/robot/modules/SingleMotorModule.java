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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoTarget;
import frc.robot.action.ActionPose;
import frc.robot.encoder.Encoder;
import frc.robot.switches.Switch;

public class SingleMotorModule implements RobotModule,AutoCloseable {
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
    double m_floatTolerance = 0.04f;
    boolean useVelocity = false;
    double currentVelocity = 0.0;
    double sustainedDriveSpeed = 0.0;

    public boolean debug = false;
    double previousDriveSpeed;
    double currentDriveSpeed;

    double previousTargetMeasurement = 0.0;
    double encoderMultiplier = 1.0;
    double reverseMultiplier = 1.0;

    private int settleCyclesRequired = 3; // tune as needed
    private int settleCount = 0;

    public boolean useFakeEncoder = !RobotBase.isReal();
    double encoderSimRate = 3.0;
    double encoderSimFactor = 0.02; // 0.005;

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
    private NetworkTableEntry targetMeasurementEntry;
    private NetworkTableEntry deltaMeasurementEntry;
    private NetworkTableEntry driveDistanceEntry;
    private NetworkTableEntry rotationCountEntry;
    private NetworkTableEntry currentDriveSpeedEntry;
    private NetworkTableEntry useVelocityEntry;
    private NetworkTableEntry velocityEntry;

    Mechanism2d mech;
    MechanismRoot2d mechRoot;
    MechanismLigament2d mechMotion;
    double endDistance = 1.0;
    double distancePerRotation = 0.3;

    // TODO 1: add velocity setpoints and modes similar to position
    public SingleMotorModule(String ModuleID, MotorController DriveMotor, double DriveSpeed, boolean Invert, Switch UpperLimit, Switch LowerLimit, Encoder Enc, double EncoderMultiplier, double ReverseMultiplier, double DistancePerRotation, double EndDistance, boolean UseVelocity) {
        moduleID = ModuleID;
        driveMotor = DriveMotor;
        driveSpeed = DriveSpeed;
        invert = Invert;

        endDistance = EndDistance;
        useVelocity = UseVelocity;

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
        targetMeasurementEntry = myTable.getEntry("targetMeasurement");
        deltaMeasurementEntry = myTable.getEntry("deltaMeasurement");
        driveDistanceEntry = myTable.getEntry("driveDistance");
        rotationCountEntry = myTable.getEntry("rotationCount");
        currentDriveSpeedEntry = myTable.getEntry("currentDriveSpeed");
        useVelocityEntry = myTable.getEntry("useVelocity");
        velocityEntry = myTable.getEntry("velocity");

        // set initial values
        invertEntry.setBoolean(invert);
        encoderMultiplierEntry.setDouble(encoderMultiplier);
        pidSetpointsEntry.setString(String.format("P: %f I: %f D: %f", pidController.getP(), pidController.getI(), pidController.getD()));
        useVelocityEntry.setBoolean(useVelocity);
        velocityEntry.setDouble(0.0);

        if (enc != null) {
            enc.setMultiplier(encoderMultiplier);
            enc.setZeroPosition();
        }

        // TODO 1: re-evaluate these calculations for velocity
        encoderSimRate = driveSpeed * encoderSimFactor;
        
        var width = 0.5;
        mech = new Mechanism2d(width, endDistance);
        mechRoot = mech.getRoot(GetModuleID(), width / 2, 0);
        mechMotion = mechRoot.append(new MechanismLigament2d("motion", 0, 90));
        SmartDashboard.putData(String.format("%s mech", moduleID), mech);  
    }

    @Override
    public void close() {
        mech.close();
    }

    public void AddActionPose(ActionPose newAction) {
        if (GetActionPose(newAction) == null) {
            actionPoses.add(newAction);
        }
    }

    public ActionPose GetActionPose(ActionPose newAction) {
        return GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action);
    }

    public ActionPose GetActionPose(String group, String location, int locationIndex, String position, String action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || "any".equalsIgnoreCase(pose.group))
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || "any".equalsIgnoreCase(pose.location))
                && (pose.position == position || "any".equalsIgnoreCase(pose.position))
                && (pose.action == action || "any".equalsIgnoreCase(pose.action))
            ) {
                System.out.printf("%s GetActionPose: Matched %s %d %d %d %s\n", moduleID, pose.group, pose.location, pose.locationIndex, pose.position, pose.action);
                return pose;
            }
        }

        return null;      
    }

    public void SetTargetActionPose(ActionPose actionPose) {
        SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
    }    

    public void SetTargetActionPose(String group, String location, int locationIndex, String position, String action) {
        var targetPose = GetActionPose(group, location, locationIndex, position, action);
        if (targetPose != null) {
            targetPoseEntry.setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));

            this.targetPose = targetPose;
            targetMeasurementEntry.setDouble(targetPose.target.Measurement);

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
            sustainedDriveSpeed = 0.0;
            myTable.getEntry("sustainedDriveSpeed").setDouble(sustainedDriveSpeed);
            currentDriveSpeed += controller.ApplyModifiers(invert ? driveSpeed : -driveSpeed) * reverseMultiplier;
            AbandonTarget();
        }
    }

    @Override
    public void ApplyValue(boolean isPressed) {
        if (isPressed) {
            sustainedDriveSpeed = 0.0;
            myTable.getEntry("sustainedDriveSpeed").setDouble(sustainedDriveSpeed);
            currentDriveSpeed += controller.ApplyModifiers(invert ? -driveSpeed : driveSpeed);
            AbandonTarget();
        }
    }

    public void EvaluateTargetPose() {
        // TODO: detect button input and bypass - how would this ever get set to non-zero at this point?
        if (targetPose != null) { //} && currentDriveSpeed != 0.0) {
            var target = targetPose.target;

            // reset sustained drive speed on new target
            sustainedDriveSpeed = 0.0;
            myTable.getEntry("sustainedDriveSpeed").setDouble(sustainedDriveSpeed);
    
            // we have a target and we're not manually applying a value, try to get to it.
            // the x axis of the position of the pose is the rotation count (distance along the motor axis)
            var targetValue = target.Measurement;
            var measurement = rotationCount;
            var targetDelta = 0.0;
            if (useVelocity)
                measurement = currentVelocity;
            targetDelta = targetValue - measurement;
            deltaMeasurementEntry.setDouble(targetDelta);
            previousTargetMeasurement = targetDelta;

            var shouldMove = (Math.abs(targetDelta) > m_floatTolerance);
            if (!shouldMove) {
                // increment global settle counter; do not abandon until threshold reached
                if (settleCount < settleCyclesRequired) {
                    settleCount++;
                    settleCountEntry.setNumber(settleCount);
                    if (useVelocity && previousDriveSpeed != 0.0) {
                        // keep our target velocity
                        sustainedDriveSpeed = previousDriveSpeed;
                        myTable.getEntry("sustainedDriveSpeed").setDouble(sustainedDriveSpeed);
                    }
                }

                if (settleCount >= settleCyclesRequired) {
                    AbandonTarget();
                }
            } else {
                if (useVelocity) {
                    // desired velocity divided by maximum achievable velocity = motor power
                    currentDriveSpeed = targetValue / (driveSpeed * 60.0);
                    System.out.printf("%s currentDriveSpeed: %f; targetValue: %f; driveSpeed: %f; driveSpeed * 60.0: %f\n", moduleID, currentDriveSpeed, targetValue, driveSpeed, driveSpeed * 60.0);
                }
                else {
                    var newSpeed = pidController.calculate(measurement, targetValue);
                    currentDriveSpeed = controller.ApplyModifiers(newSpeed);
                }

                // clamp to real values
                currentDriveSpeed = Math.max(-1.0, Math.min(1.0, currentDriveSpeed));
            }

            var driveDistance = rotationCount - previousRotationCount;
            driveDistanceEntry.setDouble(driveDistance);
        }
    }
    
    @Override
    public void ProcessState(boolean isAuto) {
        if (enc != null) {
            if (enc.isAbsolute())
                setRotationFromAbsolute();
            else {
                rotationCount = enc.getRawValue();
                currentVelocity = enc.getVelocity();
            }
        }
        rotationCountEntry.setDouble(rotationCount);
        velocityEntry.setDouble(currentVelocity);

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

        // fake adjust current angle to simulate encoder input
        if (enc != null) {
            if (useFakeEncoder) {
                enc.appendSimValueRot(currentDriveSpeed * encoderSimRate);
            }
        }
        else {
            rotationCount = rotationCount + (currentDriveSpeed * encoderSimRate);
            currentVelocity = (rotationCount - previousRotationCount) / 0.02 * 60;
        }

        mechMotion.setLength(rotationCount * distancePerRotation);

        currentDriveSpeed = sustainedDriveSpeed;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public String GetModuleID() {
        return moduleID;
    }
    
    public void OverrideTargetActionPose(ActionPose newpose) {
        targetPose = newpose;
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
                    OverrideTargetActionPose(new ActionPose("any", "any", -1, "any", "any", new AutoTarget(position)));
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
