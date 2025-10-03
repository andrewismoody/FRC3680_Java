package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.positioner.Positioner;
import frc.robot.action.*;
import frc.robot.gyro.Gyro;

public class SwerveDriveModule implements DriveModule {
    String moduleID;
    ArrayList<SwerveMotorModule> driveModules = new ArrayList<SwerveMotorModule>();
    double driveSpeed;
    double rotationSpeed;
    double rotationMultiplier = 10.0;
    ModuleController controller;
    SwerveDriveKinematics kinematics;
    boolean isFieldOriented = false;
    Gyro gyro;
    Positioner positioner;
    SwerveDriveOdometry odometry;

    Translation3d currentPosition = new Translation3d();
    Translation3d previousPosition = new Translation3d();

    double forwardSpeed = 0.0;
    double lateralSpeed = 0.0;
    double rotationAngle = 0.0;
    double angleOffset = 0.0;
    double floatTolerance;
    double deltaLimit = 0.5;

    boolean debug;
    public boolean useFakeGyro = !RobotBase.isReal();
    double currentAngle = 0.0;
    double previousAngle = 0.0;
    double fakeGyroRate = 0.3;

    boolean isZeroPressed = false;
    boolean isLockPressed = false;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    ActionPose targetPose;

    PIDController lateralPidController;
    PIDController forwardPidController;
    PIDController rotationPidController;

    // Single settle counter: require N consecutive cycles with all axes reached
    private int settleCyclesRequired = 3; // tune as needed
    private int settleCount = 0;

    NetworkTable myTable;

    // private boolean positionerHealthy = true;
    // private Translation3d lastHealthPos = new Translation3d();
    // private long lastHealthTsMs = 0L;
    // private final double posJumpLimitMeters = 2.0; // treat larger jumps as invalid
    // private final long posStaleTimeoutMs = 300;    // treat stale samples as invalid
    
    public SwerveDriveModule(String ModuleID, Gyro Gyro, Positioner Positioner, double DriveSpeed, double RotationSpeed,
            double FloatTolerance, SwerveMotorModule ... modules) {
        moduleID = ModuleID;

        var translations = new Translation2d[modules.length];
        var positions = new SwerveModulePosition[modules.length];
        var i = 0;

        myTable = NetworkTableInstance.getDefault().getTable(moduleID);

        driveSpeed = DriveSpeed;
        rotationSpeed = RotationSpeed;
        gyro = Gyro;
        positioner = Positioner;
        floatTolerance = FloatTolerance;

        // initialize modules after setting values, as modules lookup values from controller
        // TODO: maybe make this a little less brittle
        for (SwerveMotorModule module : modules) {
            module.setDriveModule(this);
            driveModules.add(module);
            translations[i] = module.modulePosition;
            positions[i] = new SwerveModulePosition(0, new Rotation2d());
            i++;
        }
        
        // TODO: might be mixing things here with drivespeed applying to drive motors and also drivespeed for the whole robot
        var posKp = 5.0; 
        var posKi = 0; //posKp * 0.10;
        var posKd = 0; //posKi * 3.0;
        lateralPidController = new PIDController(posKp, posKi, posKd);
        forwardPidController = new PIDController(posKp, posKi, posKd);

        var rotKp = 0.333;
        var rotKi = 0; // rotKp * 0.10;
        var rotKd = 0; //rotKi * 3.0;
        rotationPidController = new PIDController(rotKp, rotKi, rotKd);
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        rotationPidController.setTolerance(floatTolerance);
    
        kinematics = new SwerveDriveKinematics(translations);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getGyroAngleZ()), positions);
    }

    public String GetModuleID() {
        return moduleID;
    }

    public void ReturnToZero(boolean isPressed) {
        isZeroPressed = isPressed;

        /*
        if (isPressed) {
            System.out.println("ReturnToZero");
            for (SwerveMotorModule module : driveModules) {
                var zero = new SwerveModuleState();
                module.setAngle(zero);
                module.setSpeed(zero);
            }
        }
        */
    }

    public SwerveModuleState[] GetZeroStates() {
        var returnStates = new SwerveModuleState[driveModules.size()];

        for (int i = 0; i < driveModules.size(); i++) {
            returnStates[i] = new SwerveModuleState();
        }
        
        return returnStates;
    }

    public void LockPosition(boolean isPressed) {
        isLockPressed = isPressed;
    }

    public SwerveModuleState[] GetLockStates() {
        var returnStates = new SwerveModuleState[driveModules.size()];

        for (int i = 0; i < driveModules.size(); i++) {
            var lock = new SwerveModuleState();
            var angle = i % 2 == 0 ? 0.25 : -0.25;
            lock.angle = new Rotation2d(angle);
            returnStates[i] = lock;
        }

        return returnStates;
    }

    public void Initialize() {
        var startupAngle = useFakeGyro ? currentAngle
        : gyro.getAngle();
        myTable.getEntry("startupAngle").setDouble(startupAngle);

        // TODO: probably don't this - check and see if 180 is needed when we are blue-origin oriented.
        // angleOffset = startupAngle + 180;

        var startupPosition = positioner.GetPosition();
        myTable.getEntry("startupPosition").setString(startupPosition.toString());
        myTable.getEntry("useFakeGyro").setBoolean(useFakeGyro);
        myTable.getEntry("rotationSpeed").setDouble(rotationSpeed);
        myTable.getEntry("driveSpeed").setDouble(driveSpeed);
        myTable.getEntry("rotationPidSetpoints").setString(String.format("%f %f %f", rotationPidController.getP(), rotationPidController.getI(), rotationPidController.getD()));
        myTable.getEntry("lateralPidSetpoints").setString(String.format("%f %f %f", lateralPidController.getP(), lateralPidController.getI(), lateralPidController.getD()));
        myTable.getEntry("forwardPidSetpoints").setString(String.format("%f %f %f", forwardPidController.getP(), forwardPidController.getI(), forwardPidController.getD()));
        myTable.getEntry("fieldOriented").setBoolean(isFieldOriented);

        positioner.Initialize();

        for (SwerveMotorModule module : driveModules) {
            module.Initialize();
        }
    }

    public void SetFieldOriented(boolean value) {
        this.isFieldOriented = value;
        myTable.getEntry("fieldOriented").setBoolean(value);
    }

    public boolean IsFieldOriented() {
        return this.isFieldOriented;
    }

    // Zero all commanded outputs and optionally push zeros to hardware
    private void zeroDriveCommands() {
        // clear commanded speeds
        ProcessForwardSpeed(0.0);
        ProcessLateralSpeed(0.0);
        ProcessRotationAngle(0.0);

        // reset controllers to avoid residual outputs
        if (lateralPidController != null) lateralPidController.reset();
        if (forwardPidController != null) forwardPidController.reset();
        if (rotationPidController != null) rotationPidController.reset();
    }

    public double getGyroAngle() {
        var newAngle = useFakeGyro ? currentAngle
        // modulo the result to get rid of rotation count
        : gyro.getAngle() % 360;

        newAngle -= angleOffset;

        positioner.SetRobotOrientation("", newAngle, 0,0,0,0,0);

        return newAngle;        
    }

    public void ProcessForwardSpeed(double value) {
        forwardSpeed = value;
        myTable.getEntry("forwardSpeed").setDouble(forwardSpeed);
    }

    public void ProcessLateralSpeed(double value) {
        lateralSpeed = value;
        myTable.getEntry("lateralSpeed").setDouble(lateralSpeed);
    }

    public void ProcessRotationAngle(double value) {
        rotationAngle = value * rotationMultiplier;
        myTable.getEntry("rotationAngle").setDouble(rotationAngle);
    }

    public void StopRotation() {
        rotationAngle = getGyroAngle();
        myTable.getEntry("rotationAngle").setDouble(rotationAngle);
    }

    public void ApplyInverse(boolean isAuto) {
        // not implemented
    }

    public void ApplyValue(boolean isAuto) {
        // not implemented
    }


    public void AddActionPose(ActionPose newAction) {
        if (GetActionPose(newAction) == null) {
            actionPoses.add(newAction);
        }
    }

    public ActionPose GetActionPose(ActionPose newAction) {
        return GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action);
    }

    public ActionPose GetActionPose(Group group, Location location, int locationIndex, Position position, Action action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || pose.group == Group.Any)
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || pose.location == Location.Any)
                && (pose.position == position || pose.position == Position.Any)
                && (pose.action == action || pose.action == Action.Any)
            ) {
                return pose;
            }
        }

        return null;      
    }

    public ActionPose GetTarget() {
        return targetPose;
    }

    public void SetTargetActionPose(ActionPose actionPose) {
        SetTargetActionPose(actionPose.group, actionPose.location, actionPose.locationIndex, actionPose.position, actionPose.action);
    }      
    
    public void SetTargetActionPose(Group group, Location location, int locationIndex, Position position, Action action) {
        ActionPose actionPose = GetActionPose(group, location, locationIndex, position, action);

        if (actionPose != null) {
            targetPose = actionPose;
            myTable.getEntry("targetActionPose").setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));
            myTable.getEntry("lateralReached").setBoolean(false);
            myTable.getEntry("forwardReached").setBoolean(false);
            myTable.getEntry("rotationReached").setBoolean(false);
            
            // reset settle counter on new target
            settleCount = 0;
            myTable.getEntry("settleCount").setNumber(settleCount);
        }
    }

    private boolean isPositionerHealthy() {
        // var reason = "none";
        // long now = System.currentTimeMillis();
        // Translation3d pos = positioner.GetPosition();
        // boolean bad =
        //     Double.isNaN(pos.getX()) || Double.isNaN(pos.getY()) || Double.isNaN(pos.getZ()) ||
        //     Double.isInfinite(pos.getX()) || Double.isInfinite(pos.getY()) || Double.isInfinite(pos.getZ());

        // if (!bad) {
        //     double jump = lastHealthPos.minus(pos).getNorm();
        //     if (lastHealthTsMs != 0 && jump > posJumpLimitMeters) bad = true;
        //     if (lastHealthTsMs != 0 && (now - lastHealthTsMs) > posStaleTimeoutMs) bad = true;
        // } else
        //     reason = "NAN/Infinite";

        // if (!bad) {
        //     lastHealthPos = pos;
        //     lastHealthTsMs = now;
        // } else
        //     reason = "jump";

        // positionerHealthy = !bad;

        var positionerHealthy = positioner.IsValid();

        myTable.getEntry("positionerHealthy").setBoolean(positionerHealthy);
        return positionerHealthy;
    }

    public void EvaluateTargetPose(double newAngle) {
        var positionerHealthy = isPositionerHealthy();
        if (targetPose != null) {
            double newAngleRad = Units.degreesToRadians(newAngle);
            myTable.getEntry("newAngleRad").setDouble(newAngleRad);
            var pose = targetPose.pose;
            var position = pose.getTranslation();
            var rotation = pose.getRotation();

            var positionDelta = currentPosition.minus(position);
            var rotationDelta = newAngleRad - rotation.getZ();
            myTable.getEntry("rotationTarget").setDouble(rotation.getZ());
            myTable.getEntry("positionTarget").setString(position.toString());

            myTable.getEntry("positionDelta").setString(positionDelta.toString());
            myTable.getEntry("rotationDelta").setDouble(rotationDelta);

            var lateralReached = false;
            var forwardReached = false;
            var rotationReached = false;

            // TODO: tune PID values
            // TODO: determine if we need to adjust speed values
            if (positionerHealthy) { // don't drive if we've lost position or position is invalid
                var lateralSpeed = lateralPidController.calculate(currentPosition.getX(), position.getX());
                if (Math.abs(lateralSpeed) < floatTolerance) {
                    lateralReached = true;
                    myTable.getEntry("lateralReached").setBoolean(lateralReached);
                    ProcessLateralSpeed(0.0);
                } else {
                    ProcessLateralSpeed(lateralSpeed);
                }

                var forwardSpeed = forwardPidController.calculate(currentPosition.getY(), position.getY());
                if (Math.abs(forwardSpeed) < floatTolerance) {
                    forwardReached = true;
                    myTable.getEntry("forwardReached").setBoolean(forwardReached);
                    ProcessForwardSpeed(0.0);
                } else {
                    ProcessForwardSpeed(forwardSpeed);
                }
            } else {
                ProcessForwardSpeed(0.0);
                ProcessLateralSpeed(0.0);
            }

            var rotationSpeed = rotationPidController.calculate(newAngleRad, rotation.getZ());
            if (Math.abs(rotationSpeed) < floatTolerance) {
                rotationReached = true;
                myTable.getEntry("rotationReached").setBoolean(rotationReached);
                ProcessRotationAngle(0.0);
            } else {
                ProcessRotationAngle(rotationSpeed);
            }

            if (lateralReached && forwardReached && rotationReached) {
                // increment global settle counter; do not abandon until threshold reached
                if (settleCount < settleCyclesRequired) {
                    settleCount++;
                }
                if (settleCount >= settleCyclesRequired) {
                    AbandonTarget();
                }
            } else {
                // any axis out of tolerance resets the settle window
                settleCount = 0;
            }
            myTable.getEntry("settleCount").setNumber(settleCount);
        }
    }

    public void AbandonTarget() {
        targetPose = null;
        
        myTable.getEntry("targetActionPose").setString("none");
        myTable.getEntry("lateralReached").unpublish();
        myTable.getEntry("forwardReached").unpublish();
        myTable.getEntry("rotationReached").unpublish();
        myTable.getEntry("targetDelta").unpublish();
        myTable.getEntry("rotationDelta").unpublish();
    }

    public void ProcessState(boolean isAuto) {
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.htm
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
        // https://www.chiefdelphi.com/t/set-motor-position-with-encoder/152088/3

        // Invert the Gyro angle because it rotates opposite of the robot steering, then wrap it to a positive value
        double newAngle = ((-getGyroAngle() % 360.0) + 360.0) % 360.0;
        myTable.getEntry("actualAngle").setDouble(newAngle);

        EvaluateTargetPose(newAngle);

        // set the chassis speed object according to current controller values
        double forwardSpeed = this.forwardSpeed * controller.ApplyModifiers(driveSpeed);
        double lateralSpeed = this.lateralSpeed * controller.ApplyModifiers(driveSpeed);
        double thisRotationSpeed = controller.ApplyModifiers(rotationAngle); //rotationAngle; // * controller.ApplyModifiers(this.rotationSpeed);
        myTable.getEntry("rotationSpeed").setDouble(thisRotationSpeed);

        ChassisSpeeds speeds = isFieldOriented ?
            ChassisSpeeds.fromFieldRelativeSpeeds(lateralSpeed, forwardSpeed, thisRotationSpeed, Rotation2d.fromDegrees(newAngle))
            : new ChassisSpeeds(lateralSpeed, forwardSpeed, thisRotationSpeed);

        SwerveModuleState[] moduleStates;

        boolean optimize = true;
        if (isLockPressed) {
            moduleStates = GetLockStates();
            optimize = false;
        } else if (isZeroPressed) {
            optimize = false;
            moduleStates = GetZeroStates();
        } else
            moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveModulePosition[] positions = new SwerveModulePosition[driveModules.size()];
        double[] driveSpeeds = new double[4]; //always 4 because of dashboard setup

        for (int i = 0; i < driveModules.size(); i++) {
            SwerveMotorModule module = driveModules.get(i);
            module.updateModuleValues(moduleStates[i], optimize);
            positions[i] = module.getPosition();
            if (i < driveSpeeds.length)
                driveSpeeds[i] = module.getSpeed();
        }

        odometry.update(Rotation2d.fromDegrees(newAngle), positions);

        var primaryModule = driveModules.get(0);
        if (primaryModule != null) {
            // calculate and store current field position and rotation
            // var centerOffset = new Translation2d(Math.cos(getGyroAngle()) * primaryModule.modulePosition.getX(),
            //         Math.sin(getGyroAngle()) * primaryModule.modulePosition.getY());
            currentPosition = GetPosition().getTranslation(); //primaryModule.currentPosition.minus(centerOffset);
            myTable.getEntry("currentPosition").setString(currentPosition.toString());
            if ((Math.abs(previousPosition.minus(currentPosition).getX()) > floatTolerance
                    || Math.abs(previousPosition.minus(currentPosition).getY()) > floatTolerance)
                    && Math.abs(previousPosition.minus(currentPosition).getNorm()) < deltaLimit) {
                previousPosition = currentPosition;
            }
        }

        // update fake gyro angle
        currentAngle += thisRotationSpeed * fakeGyroRate;
        myTable.getEntry("fakeAngle").setDouble(currentAngle);
        previousAngle = currentAngle;

        // update dashboard
        SmartDashboard.putNumberArray("RobotDrive Motors", new double[] {driveModules.get(0).getSpeed(), driveModules.get(1).getSpeed(), 0.0, 0.0});
        //SmartDashboard.putNumberArray("My Motors", new double[] {driveModules.get(0).getSpeed(), driveModules.get(1).getSpeed(), 0.0, 0.0});
        //System.out.printf("leftFront speed: %f\n", driveModules.get(0).getSpeed()); // , driveModules.get(1).getSpeed(), 0.0, 0.0});

        zeroDriveCommands();
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    @Override
    public Pose3d GetPosition() {
        return new Pose3d(positioner.GetPosition(), new Rotation3d(0, 0, getGyroAngle()));
        // return new Translation3d(currentPosition.getX(), currentPosition.getY(), currentAngle);
    }
}
