package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;

public class SwerveDriveModule implements DriveModule {
    String moduleID;
    SwerveMotorModule leftFrontDriveModule;
    SwerveMotorModule rightRearDriveModule;
    double driveSpeed;
    double rotationSpeed;
    ModuleController controller;
    SwerveDriveKinematics kinematics;
    boolean isFieldOriented;
    AnalogGyro gyro;

    Translation2d currentPosition = new Translation2d();
    Translation2d previousPosition = new Translation2d();
  
    double forwardSpeed = 0.0;
    double lateralSpeed = 0.0;
    double rotationAngle = 0.0;
    double previousForwardSpeed = 0.0;
    double previousLateralSpeed = 0.0;
    double previousRotationAngle = 0.0;
    double floatTolerance;

    boolean debug;
    boolean useFakeGyro = true;
    double currentAngle = 0.0;
    double fakeGyroRate = 0.3;
  
    public SwerveDriveModule(String ModuleID, SwerveMotorModule LeftFrontDriveModule, SwerveMotorModule RightRearDriveModule, AnalogGyro Gyro, double DriveSpeed, double RotationSpeed, boolean IsFieldOriented, double FloatTolerance) {
        moduleID = ModuleID;
        leftFrontDriveModule = LeftFrontDriveModule;
        leftFrontDriveModule.setDriveModule(this);
        rightRearDriveModule = RightRearDriveModule;
        rightRearDriveModule.setDriveModule(this);
        driveSpeed = DriveSpeed;
        rotationSpeed = RotationSpeed;
        isFieldOriented = IsFieldOriented;
        gyro = Gyro;
        floatTolerance = FloatTolerance;
        kinematics = new SwerveDriveKinematics(leftFrontDriveModule.modulePosition, rightRearDriveModule.modulePosition);
    }

    public void Initialize() {

    }

    public void ProcessForwardSpeed(double value) {
        forwardSpeed = value;

        if (debug && forwardSpeed != previousForwardSpeed)
            System.out.printf("%s forwardSpeed: %f\n", moduleID, forwardSpeed);
        
        previousForwardSpeed = forwardSpeed;
    }
  
    public void ProcessLateralSpeed(double value) {
        lateralSpeed = value;

        if (debug && lateralSpeed != previousLateralSpeed)
            System.out.printf("%s lateralSpeed: %f\n", moduleID, lateralSpeed);
        
        previousLateralSpeed = lateralSpeed;
    }
  
    public void ProcessRotationAngle(double value) {
        rotationAngle = value;

        if (debug && rotationAngle != previousRotationAngle)
            System.out.printf("%s rotationAngle: %f\n", moduleID, rotationAngle);
        
        previousRotationAngle = rotationAngle;
    }
  
    public void ProcessState() {
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.htm
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
        // https://www.chiefdelphi.com/t/set-motor-position-with-encoder/152088/3
        double newAngle = useFakeGyro ?
            currentAngle
            :
            gyro.getAngle()
        ;
  
        // set the chassis speed object according to current controller values
        double forwardSpeed = this.forwardSpeed * controller.ApplyModifiers(driveSpeed);
        double lateralSpeed = this.lateralSpeed * controller.ApplyModifiers(driveSpeed);
        double rotationSpeed = rotationAngle * controller.ApplyModifiers(this.rotationSpeed);

        ChassisSpeeds speeds = isFieldOriented ?
        new ChassisSpeeds(lateralSpeed, forwardSpeed, rotationSpeed)
        :
        ChassisSpeeds.fromFieldRelativeSpeeds(lateralSpeed, forwardSpeed, rotationSpeed, Rotation2d.fromDegrees(newAngle))
        ;

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        leftFrontDriveModule.updateModuleValues(moduleStates);
        rightRearDriveModule.updateModuleValues(moduleStates);

        // calculate and store current field position and rotation
        var centerOffset = new Translation2d(Math.cos(gyro.getAngle()) * leftFrontDriveModule.modulePosition.getX(), Math.sin(gyro.getAngle()) * leftFrontDriveModule.modulePosition.getY());
        currentPosition = leftFrontDriveModule.currentPosition.minus(centerOffset);
        if ((Math.abs(previousPosition.minus(currentPosition).getX()) > floatTolerance || Math.abs(previousPosition.minus(currentPosition).getY()) > floatTolerance) && debug) {
            System.out.printf("%s ProcessState: currentPosition: %s\n", moduleID, currentPosition);
            previousPosition = currentPosition;
        }

        // update fake gyro angle
        currentAngle += rotationSpeed * fakeGyroRate;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }
}
