package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.encoder.Encoder;
import frc.robot.switches.Switch;

public class SingleMotorModule implements RobotModule {
    String moduleID;
    MotorController driveMotor;
    double driveSpeed;
    boolean invert;
    ModuleController controller;

    Encoder enc;
    Switch upperLimit;
    Switch lowerLimit;

    public boolean debug = false;
    double previousDriveSpeed;
    double currentDriveSpeed;

    public SingleMotorModule(String ModuleID, MotorController DriveMotor, double DriveSpeed, boolean Invert, Switch UpperLimit, Switch LowerLimit, Encoder Enc) {
        moduleID = ModuleID;
        driveMotor = DriveMotor;
        driveSpeed = DriveSpeed;
        invert = Invert;

        upperLimit = UpperLimit;
        lowerLimit = LowerLimit;
        enc = Enc;
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
        
        if (currentDriveSpeed > 0 && !upperLimit.GetState() ||
            currentDriveSpeed < 0 && !lowerLimit.GetState()) {
                driveMotor.setVoltage(invert ? -currentDriveSpeed : currentDriveSpeed);
        } else {
            if (debug) {
                System.out.println("limit reached, not driving motor");
            }
            driveMotor.setVoltage(0);
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
        if (enc != null)
            return new Pose3d(new Translation3d(enc.getDistance(), 0, 0), new Rotation3d());
        
        return new Pose3d();
    }
}
