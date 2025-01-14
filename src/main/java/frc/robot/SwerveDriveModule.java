package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.gyro.Gyro;

public class SwerveDriveModule implements DriveModule {
    String moduleID;
    ArrayList<SwerveMotorModule> driveModules = new ArrayList<SwerveMotorModule>();
    double driveSpeed;
    double rotationSpeed;
    double rotationMultiplier = 10.0;
    ModuleController controller;
    SwerveDriveKinematics kinematics;
    boolean isFieldOriented;
    Gyro gyro;
    SwerveDriveOdometry odometry;

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
    public boolean useFakeGyro = !RobotBase.isReal();
    double currentAngle = 0.0;
    double previousAngle = 0.0;
    double fakeGyroRate = 0.3;

    public SwerveDriveModule(String ModuleID, Gyro Gyro, double DriveSpeed, double RotationSpeed,
            boolean IsFieldOriented, double FloatTolerance, SwerveMotorModule ... modules) {
        moduleID = ModuleID;

        var translations = new Translation2d[modules.length];
        var positions = new SwerveModulePosition[modules.length];
        var i = 0;

        for (SwerveMotorModule module : modules) {
            module.setDriveModule(this);
            driveModules.add(module);
            translations[i] = module.modulePosition;
            positions[i] = new SwerveModulePosition(0, new Rotation2d());
            i++;
        }

        driveSpeed = DriveSpeed;
        rotationSpeed = RotationSpeed;
        isFieldOriented = IsFieldOriented;
        gyro = Gyro;
        floatTolerance = FloatTolerance;

        kinematics = new SwerveDriveKinematics(translations);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getGyroAngleZ()), positions);
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
        rotationAngle = value * rotationMultiplier;

        if (debug && rotationAngle != previousRotationAngle)
            System.out.printf("%s rotationAngle: %f\n", moduleID, rotationAngle);

        previousRotationAngle = rotationAngle;
    }

    public void StopRotation() {
        rotationAngle = useFakeGyro ? currentAngle
                : gyro.getAngle();

        if (debug && rotationAngle != previousRotationAngle)
            System.out.printf("%s rotationAngle: %f\n", moduleID, rotationAngle);

        previousRotationAngle = rotationAngle;
    }

    public void ProcessState() {
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.htm
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
        // https://www.chiefdelphi.com/t/set-motor-position-with-encoder/152088/3
        double newAngle = useFakeGyro ? currentAngle
                : gyro.getAngle();

        // set the chassis speed object according to current controller values
        double forwardSpeed = this.forwardSpeed * controller.ApplyModifiers(driveSpeed);
        double lateralSpeed = this.lateralSpeed * controller.ApplyModifiers(driveSpeed);
        double thisRotationSpeed = controller.ApplyModifiers(rotationAngle); //rotationAngle; // * controller.ApplyModifiers(this.rotationSpeed);
        // if (debug)
        //     System.out.printf("%s; thisRotationSpeed: %s\n", moduleID, thisRotationSpeed);

        ChassisSpeeds speeds = isFieldOriented ?
            ChassisSpeeds.fromFieldRelativeSpeeds(lateralSpeed, forwardSpeed, thisRotationSpeed, Rotation2d.fromDegrees(newAngle))
            : new ChassisSpeeds(lateralSpeed, forwardSpeed, thisRotationSpeed);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        var i = 0;
        SwerveModulePosition[] positions = new SwerveModulePosition[driveModules.size()];
        double[] driveSpeeds = new double[4]; //always 4 because of dashboard setup

        for (SwerveMotorModule module : driveModules) {
            module.updateModuleValues(moduleStates[i]);
            positions[i] = module.getPosition();
            if (i < driveSpeeds.length)
                driveSpeeds[i] = module.getSpeed();
            i++;
        }

        odometry.update(Rotation2d.fromDegrees(newAngle), positions);

        var primaryModule = driveModules.get(0);
        if (primaryModule != null) {
            // calculate and store current field position and rotation
            var centerOffset = new Translation2d(Math.cos(gyro.getAngle()) * primaryModule.modulePosition.getX(),
                    Math.sin(gyro.getAngle()) * primaryModule.modulePosition.getY());
            currentPosition = primaryModule.currentPosition.minus(centerOffset);
            if ((Math.abs(previousPosition.minus(currentPosition).getX()) > floatTolerance
                    || Math.abs(previousPosition.minus(currentPosition).getY()) > floatTolerance) && debug) {
                System.out.printf("%s ProcessState: currentPosition: %s\n", moduleID, currentPosition);
                previousPosition = currentPosition;
            }
        }

        // update fake gyro angle
        currentAngle += thisRotationSpeed * fakeGyroRate;
        if (Math.abs(previousAngle - currentAngle) > floatTolerance) {
            System.out.printf("currentAngle: %f; previousAngle: %f\n", currentAngle, previousAngle);
            previousAngle = currentAngle;
        }

        // update dashboard
        SmartDashboard.putNumberArray("RobotDrive Motors", new double[] {driveModules.get(0).getSpeed(), driveModules.get(1).getSpeed(), 0.0, 0.0});
        //SmartDashboard.putString("DB/String 9", String.valueOf(gyro.getAngle()));
        //SmartDashboard.putNumberArray("My Motors", new double[] {driveModules.get(0).getSpeed(), driveModules.get(1).getSpeed(), 0.0, 0.0});
        //System.out.printf("leftFront speed: %f\n", driveModules.get(0).getSpeed()); // , driveModules.get(1).getSpeed(), 0.0, 0.0});
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public Translation3d GetPosition() {
        return new Translation3d(currentPosition.getX(), currentPosition.getY(), currentAngle);
    }
}
