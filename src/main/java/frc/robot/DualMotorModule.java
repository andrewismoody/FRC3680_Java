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
    
    public void ProcessState(boolean value) {
        var currentDriveSpeed = 0.0;

        if (value)
            currentDriveSpeed = controller.ApplyModifiers(driveSpeed);

        if (debug && previousDriveSpeed != currentDriveSpeed) {
            System.out.printf("%s currentDriveSpeed %f\n", moduleID, currentDriveSpeed);
            previousDriveSpeed = currentDriveSpeed;
        }
        
        if (currentDriveSpeed > 0 && !upperLimit.GetState() ||
            currentDriveSpeed < 0 && !lowerLimit.GetState()) {
                leftDriveMotor.setVoltage(invertLeft ? -currentDriveSpeed : currentDriveSpeed);
                rightDriveMotor.setVoltage(invertRight ? -currentDriveSpeed : currentDriveSpeed);
        } else {
            if (debug) {
                System.out.println("limit reached, not driving motor");
            }
            leftDriveMotor.setVoltage(0);
            rightDriveMotor.setVoltage(0);
        }
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
