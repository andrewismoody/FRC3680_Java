package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DualMotorModule implements RobotModule {
    String moduleID;
    MotorController leftDriveMotor;
    MotorController rightDriveMotor;
    double driveSpeed;
    boolean invertLeft;
    boolean invertRight;
    ModuleController controller;

    boolean debug = true;

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
        var mySpeed = 0.0;

        if (value) {
            mySpeed = controller.ApplyModifiers(driveSpeed);

            if (debug)
                System.out.printf("%s set speed %f\n", moduleID, mySpeed);
        }

        leftDriveMotor.set(invertLeft ? -mySpeed : mySpeed);
        rightDriveMotor.set(invertRight ? -mySpeed : mySpeed);
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }   
}
