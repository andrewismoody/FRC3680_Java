package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

    double m_forwardSpeed = 0.0;
    double m_lateralSpeed = 0.0;
    double m_rotationAngle = 0.0;
  
    public SwerveDriveModule(String ModuleID, SwerveMotorModule LeftFrontDriveModule, SwerveMotorModule RightRearDriveModule, AnalogGyro Gyro, double DriveSpeed, double RotationSpeed, boolean IsFieldOriented) {
        moduleID = ModuleID;
        leftFrontDriveModule = LeftFrontDriveModule;
        leftFrontDriveModule.setDriveModule(this);
        rightRearDriveModule = RightRearDriveModule;
        rightRearDriveModule.setDriveModule(this);
        driveSpeed = DriveSpeed;
        rotationSpeed = RotationSpeed;
        isFieldOriented = IsFieldOriented;
        gyro = Gyro;
        kinematics = new SwerveDriveKinematics(leftFrontDriveModule.modulePosition, rightRearDriveModule.modulePosition);
    }

    public void Initialize() {

    }

    public void ProcessForwardSpeed(double value) {
        m_forwardSpeed = value;
    }
  
    public void ProcessLateralSpeed(double value) {
        m_lateralSpeed = value;
    }
  
    public void ProcessRotationAngle(double value) {
        m_rotationAngle = value;
    }
  
    public void ProcessState() {
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.htm
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
        // https://www.chiefdelphi.com/t/set-motor-position-with-encoder/152088/3

        // set the chassis speed object according to current controller values
        double forwardSpeed = m_forwardSpeed * controller.ApplyModifiers(driveSpeed);
        double lateralSpeed = m_lateralSpeed * controller.ApplyModifiers(driveSpeed);
        double rotationSpeed = m_rotationAngle * controller.ApplyModifiers(this.rotationSpeed);

        ChassisSpeeds speeds = isFieldOriented ?
        new ChassisSpeeds(lateralSpeed, forwardSpeed, rotationSpeed)
        :
        ChassisSpeeds.fromFieldRelativeSpeeds(lateralSpeed, forwardSpeed, rotationSpeed, Rotation2d.fromDegrees(gyro.getAngle()))
        ;

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        
        leftFrontDriveModule.updateModuleValues(moduleStates);
        rightRearDriveModule.updateModuleValues(moduleStates);
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }
}
