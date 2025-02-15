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

    double fullRotation = 6.28;
    double previousEncValue = 0.0;
    double rotationCount = 0.0;
    double previousRotationCount = 0.0;
    double m_floatTolerance = 0.5f;

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

        if (enc != null)
            previousEncValue = enc.getDistance();
    }

    public void Initialize() {

    }

    @Override
    public void ApplyInverse(boolean value) {
        if (value) {
            currentDriveSpeed += controller.ApplyModifiers(-driveSpeed);
            System.out.printf("%s: ApplyInverse; -driveSpeed %f; currentDriveSpeed %f\n", moduleID, -driveSpeed, currentDriveSpeed);
        }
    }

    @Override
    public void ApplyValue(boolean value) {
        if (value) {
            currentDriveSpeed += controller.ApplyModifiers(driveSpeed);
            System.out.printf("%s: ApplyValue; driveSpeed %f; currentDriveSpeed %f\n", moduleID, driveSpeed, currentDriveSpeed);
        }
    }
    
    @Override
    public void ProcessState(boolean isAuto) {
        if (debug && Math.abs(previousDriveSpeed - currentDriveSpeed) > m_floatTolerance) {
            System.out.printf("%s currentDriveSpeed %f\n", moduleID, currentDriveSpeed);
            previousDriveSpeed = currentDriveSpeed;
        }

        if ((currentDriveSpeed > 0 && (upperLimit == null || !upperLimit.GetState())) ||
            (currentDriveSpeed < 0 && (lowerLimit == null || !lowerLimit.GetState()))) {
                System.out.printf("%s: currentDriveSpeed %f\n", moduleID, invert ? -currentDriveSpeed : currentDriveSpeed);
                driveMotor.set(invert ? -currentDriveSpeed : currentDriveSpeed);
        } else {
            if (debug  && currentDriveSpeed != 0.0) {
                System.out.println("limit reached, not driving motor");
            }
            driveMotor.set(0);
        }

        if (enc != null) {
            if (lowerLimit != null && lowerLimit.GetState()) {
                rotationCount = 0.0;
            }
            else {
                rotationCount += (previousEncValue - enc.getDistance());
            }

            previousEncValue = enc.getDistance();
        }

        if (Math.abs(rotationCount - previousRotationCount) > m_floatTolerance && debug) {
            System.out.printf("%s: Rotation Count %f\n", moduleID, rotationCount);
            previousRotationCount = rotationCount;
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
        if (enc != null)
            return new Pose3d(new Translation3d(enc.getDistance(), 0, 0), new Rotation3d());
        
        return new Pose3d();
    }
}
