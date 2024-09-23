package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class SingleMotorModule implements RobotModule {
    String moduleID;
    MotorController driveMotor;
    double driveSpeed;
    boolean invert;
    ModuleController controller;

    boolean debug = true;

    public SingleMotorModule(String ModuleID, MotorController DriveMotor, double DriveSpeed, boolean Invert) {
        moduleID = ModuleID;
        driveMotor = DriveMotor;
        driveSpeed = DriveSpeed;
        invert = Invert;
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
            
        driveMotor.set(invert ? -mySpeed : mySpeed);
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }
}
