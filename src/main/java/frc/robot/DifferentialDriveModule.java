package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DifferentialDriveModule implements DriveModule {
    String moduleID;

    ModuleController controller;
    DifferentialDrive driveController;
    MotorController leftMotor;
    MotorController rightMotor;

    double forwardSpeed = 0.0;
    double rotationAngle = 0.0;

    public boolean debugAngle = false;
    public boolean debugSpeed = false;

    double currentForwardSpeed = 0.0;
    double currentRotationAngle = 0.0;
    double previousForwardSpeed = 0.0;
    double previousRotationAngle = 0.0;

    public DifferentialDriveModule(String ModuleID, MotorController LeftMotor, MotorController RightMotor) {
        moduleID = ModuleID;
        leftMotor = LeftMotor;
        rightMotor = RightMotor;

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotor.setInverted(true);

        driveController = new DifferentialDrive(leftMotor::set, rightMotor::set);
        driveController.setSafetyEnabled(false);
    }

    public void ProcessState() {
        currentForwardSpeed = controller.ApplyModifiers(forwardSpeed);
        if (debugSpeed && forwardSpeed != previousForwardSpeed) {
            System.out.printf("%s forwardSpeed: %f\n", moduleID, forwardSpeed);
            previousForwardSpeed = forwardSpeed;
        }

        currentRotationAngle = controller.ApplyModifiers(rotationAngle);
        if (debugAngle && rotationAngle != previousRotationAngle) {
            System.out.printf("%s rotationAngle: %f\n", moduleID, rotationAngle);
            previousRotationAngle = rotationAngle;
        }

        driveController.arcadeDrive(currentForwardSpeed, currentRotationAngle);

        // update dashboard
        SmartDashboard.putNumberArray("RobotDrive Motors", new double[]{leftMotor.get(), rightMotor.get()});
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
