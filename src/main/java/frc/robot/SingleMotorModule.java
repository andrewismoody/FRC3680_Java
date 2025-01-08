package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SingleMotorModule implements RobotModule {
    String moduleID;
    MotorController driveMotor;
    double driveSpeed;
    boolean invert;
    ModuleController controller;

    public boolean debug = false;
    double previousDriveSpeed;
    double currentDriveSpeed;

    public SingleMotorModule(String ModuleID, MotorController DriveMotor, double DriveSpeed, boolean Invert) {
        moduleID = ModuleID;
        driveMotor = DriveMotor;
        driveSpeed = DriveSpeed;
        invert = Invert;
    }

    public void Initialize() {

    }
    
    public void ProcessState(boolean value) {
        currentDriveSpeed = 0.0;

        if (value)
            currentDriveSpeed = controller.ApplyModifiers(driveSpeed);

        if (debug && previousDriveSpeed != currentDriveSpeed) {
            System.out.printf("%s currentDriveSpeed %f\n", moduleID, currentDriveSpeed);
            previousDriveSpeed = currentDriveSpeed;
        }
            
        driveMotor.set(invert ? -currentDriveSpeed : currentDriveSpeed);
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public String GetModuleID() {
        return moduleID;
    }
}
