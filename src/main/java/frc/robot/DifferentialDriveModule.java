package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DifferentialDriveModule implements DriveModule {
    String moduleID;

    ModuleController controller;
    DifferentialDrive driveController;
    MotorController leftMotor;
    MotorController rightMotor;

    double forwardSpeed = 0.0;
    double rotationAngle = 0.0;

    boolean debug = true;

    public DifferentialDriveModule(String ModuleID, MotorController LeftMotor, MotorController RightMotor) {
        moduleID = ModuleID;
        leftMotor = LeftMotor;
        rightMotor = RightMotor;

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotor.setInverted(true);

        driveController = new DifferentialDrive(leftMotor::set, rightMotor::set);
    }

    public void ProcessState() {
        if (debug && forwardSpeed != 0.0)
            System.out.printf("%s forwardSpeed: %f\n", moduleID, forwardSpeed);

        if (debug && rotationAngle != 0.0)
            System.out.printf("%s rotationAngle: %f\n", moduleID, rotationAngle);

            driveController.arcadeDrive(forwardSpeed, rotationAngle);
    }

    public void Initialize() {

    }

    public void ProcessForwardSpeed(double value) {
        forwardSpeed = value;
    }
  
    public void ProcessLateralSpeed(double value) {
    }
  
    public void ProcessRotationAngle(double value) {
        rotationAngle = value;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }
    
}
