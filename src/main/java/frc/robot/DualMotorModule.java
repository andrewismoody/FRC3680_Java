package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

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

    public DualMotorModule(String ModuleID, MotorController LeftDriveMotor, MotorController RightDriveMotor, double DriveSpeed, boolean InvertLeft, boolean InvertRight) {
        moduleID = ModuleID;
        leftDriveMotor = LeftDriveMotor;
        rightDriveMotor = RightDriveMotor;
        driveSpeed = DriveSpeed;
        invertLeft = InvertLeft;
        invertRight = InvertRight;
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

        leftDriveMotor.set(invertLeft ? -currentDriveSpeed : currentDriveSpeed);
        rightDriveMotor.set(invertRight ? -currentDriveSpeed : currentDriveSpeed);
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
}
