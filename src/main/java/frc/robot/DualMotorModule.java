package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.encoder.Encoder;
import frc.robot.switches.Switch;

public class DualMotorModule implements RobotModule {
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

    double previousleftEncValue = 0.0;
    double previousRightEncValue = 0.0;

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
        
    }

    @Override
    public void ApplyInverse(boolean value) {
        if (value) {
            System.out.printf("%s: ApplyInverse\n", moduleID);
            currentDriveSpeed += controller.ApplyModifiers(-driveSpeed);
        }
    }

    @Override
    public void ApplyValue(boolean value) {
        if (value) {
            System.out.printf("%s: ApplyValue\n", moduleID);
            currentDriveSpeed += controller.ApplyModifiers(driveSpeed);
        }  
    }
    
    public void ProcessState(boolean isAuto) {
        if (debug && previousDriveSpeed != currentDriveSpeed) {
            System.out.printf("%s currentDriveSpeed %f\n", moduleID, currentDriveSpeed);
            previousDriveSpeed = currentDriveSpeed;
        }
        
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
        }

        if (lowerLimit.GetState()) {
            leftRotationCount = 0.0;
            rightRotationCount = 0.0;
        }
        else {
            if (leftEnc != null)
                leftRotationCount += fullRotation - (previousleftEncValue - leftEnc.getDistance());
            if (rightEnc != null)
                rightRotationCount += fullRotation - (previousRightEncValue - rightEnc.getDistance());
        }

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

    public void ApproachTarget(Pose3d TargetPose) {
        // TODO: Implement this.
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
}
