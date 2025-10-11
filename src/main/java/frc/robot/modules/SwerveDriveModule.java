package frc.robot.modules;

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
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.positioner.Positioner;
import frc.robot.positioner.LimelightHelpers.PoseEstimate;
import frc.robot.z2025.Robot;
import frc.robot.action.Group;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;
import frc.robot.gyro.Gyro;
import frc.robot.misc.Utility;
import frc.robot.misc.Utility.SwervePosition;

public class SwerveDriveModule implements DriveModule {
    String moduleID;
    SwerveMotorModule[] driveModules;
    double driveSpeed;
    double driveRatio;
    double rotationSpeed;
    double rotationMultiplier = 5.0;
    ModuleController controller;
    SwerveDriveKinematics kinematics;
    boolean isFieldOriented = false;
    Gyro gyro;
    Positioner positioner;
    SwerveDriveOdometry odometry;

    Translation3d currentPosition = Translation3d.kZero;
    Translation3d previousPosition = currentPosition;
    double previousRotationSpeed = 0.0;
    double seekRotation = 0.0;
    double seekStart = 0.0;
    boolean seekReversed = false;
    boolean positionInitialized = false;

    double forwardSpeed = 0.0;
    double lateralSpeed = 0.0;
    double rotationAngle = 0.0;
    double angleOffset = 0.0;
    double floatTolerance;
    double deltaLimit = 0.5;
    double frameNorm = 0.0;

    boolean debug;
    public boolean useFakeGyro = !RobotBase.isReal();
    double currentAngle = 0.0;
    double previousAngle = 0.0;
    double fakeGyroRate = 0.6;
    Pose3d fakeCameraPose = new Pose3d(new Translation3d(0, -0.28, 0.51), new Rotation3d(new Rotation2d(1.566)));

    boolean isZeroPressed = false;
    boolean isLockPressed = false;
    boolean wasFieldOrientedPressed = false;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    ActionPose targetPose;

    PIDController lateralPidController;
    PIDController forwardPidController;
    PIDController rotationPidController;

    // Single settle counter: require N consecutive cycles with all axes reached
    private int settleCyclesRequired = 3; // tune as needed
    private int settleCount = 0;

    private boolean wroteForwardThisTick = false;
    private boolean wroteLateralThisTick = false;
    private boolean wroteRotationThisTick = false;

    SwerveDrivePoseEstimator poseEstimator;
    Field2d fieldPosition;

    NetworkTable myTable;

    // Cached NT entries
    private NetworkTableEntry startupAngleEntry;
    private NetworkTableEntry useFakeGyroEntry;
    private NetworkTableEntry rotationSpeedEntry;
    private NetworkTableEntry driveSpeedEntry;
    private NetworkTableEntry rotationPidSetpointsEntry;
    private NetworkTableEntry lateralPidSetpointsEntry;
    private NetworkTableEntry forwardPidSetpointsEntry;
    private NetworkTableEntry fieldOrientedEntry;
    private NetworkTableEntry forwardSpeedEntry;
    private NetworkTableEntry lateralSpeedEntry;
    private NetworkTableEntry rotationAngleEntry;
    private NetworkTableEntry targetActionPoseEntry;
    private NetworkTableEntry lateralReachedEntry;
    private NetworkTableEntry forwardReachedEntry;
    private NetworkTableEntry rotationReachedEntry;
    private NetworkTableEntry settleCountEntry;
    private NetworkTableEntry positionerHealthyEntry;
    private NetworkTableEntry positionHealthReasonEntry;
    private NetworkTableEntry newAngleRadEntry;
    private NetworkTableEntry positionDeltaEntry;
    private NetworkTableEntry positionTargetEntry;
    private NetworkTableEntry rotationDeltaEntry;
    private NetworkTableEntry rotationTargetEntry;
    private NetworkTableEntry lookTargetAngEntry;
    private NetworkTableEntry lookTargetPosEntry;
    private NetworkTableEntry currentGyroAngleEntry;
    private NetworkTableEntry currentPositionEntry;
    private NetworkTableEntry fakeAngleEntry;
    private NetworkTableEntry targetDeltaEntry;
    StructPublisher<Pose3d> currentPosePublisher;
    StructArrayPublisher<SwerveModuleState> swerveModuleReqestPublisher;
    StructArrayPublisher<SwerveModuleState> swerveModuleCommandedPublisher;
    StructPublisher<ChassisSpeeds> chassisSpeedsPublisher;

    public SwerveDriveModule(String ModuleID, Gyro Gyro, Positioner Positioner, double DriveSpeed, double DriveRatio, double RotationSpeed,
            double FloatTolerance, SwerveMotorModule ... modules) {
        moduleID = ModuleID;

        var translations = new Translation2d[modules.length];
        var positions = new SwerveModulePosition[modules.length];

        myTable = NetworkTableInstance.getDefault().getTable(moduleID);

        driveSpeed = DriveSpeed;
        driveRatio = DriveRatio;
        rotationSpeed = RotationSpeed;
        gyro = Gyro;
        positioner = Positioner;
        floatTolerance = FloatTolerance;

        // initialize modules after setting values, as modules lookup values from controller
        // TODO: maybe make this a little less brittle
        driveModules = new SwerveMotorModule[modules.length];
        for (SwerveMotorModule module : modules) {
            var i = module.GetSwervePosition().getValue();
            driveModules[i] = module;
            module.setDriveModule(this);
            
            translations[i] = module.modulePosition;
            positions[i] = new SwerveModulePosition(0, new Rotation2d());
        }

        var frameNormStart = modules[SwervePosition.LeftFront.getValue()].modulePosition;
        frameNorm = frameNormStart.getNorm();
        
        var posKp = 2.0; 
        var posKi = 0; //posKp * 0.10;
        var posKd = 0; //posKi * 3.0;
        lateralPidController = new PIDController(posKp, posKi, posKd);
        forwardPidController = new PIDController(posKp, posKi, posKd);

        // bumping this up to 1.0 after adjusting the rotation multiplier from processrotation
        var rotKp = 1.0;
        var rotKi = 0; // rotKp * 0.10;
        var rotKd = 0; //rotKi * 3.0;
        rotationPidController = new PIDController(rotKp, rotKi, rotKd);
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        rotationPidController.setTolerance(floatTolerance);

        kinematics = new SwerveDriveKinematics(translations);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getInvertedGyroValue()), positions);
        fieldPosition = new Field2d();

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(getInvertedGyroValue()), positions, Pose2d.kZero);
    }

    public String GetModuleID() {
        return moduleID;
    }

    public void SetDebug(boolean value) {
        debug = value;
    }

    public boolean GetDebug() {
        return debug;
    }

    public void ReturnToZero(boolean isPressed) {
        isZeroPressed = isPressed;
    }

    public SwerveModuleState[] GetZeroStates() {
        var returnStates = new SwerveModuleState[driveModules.length];

        for (SwerveMotorModule module : driveModules) {
            var i = module.GetSwervePosition().getValue();
            returnStates[i] = new SwerveModuleState();
        }
        
        return returnStates;
    }

    public void LockPosition(boolean isPressed) {
        isLockPressed = isPressed;
    }

    public SwerveModuleState[] GetLockStates() {
        var returnStates = new SwerveModuleState[driveModules.length];

        for (SwerveMotorModule module : driveModules) {
            var i = module.GetSwervePosition().getValue();
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
        
        // instantiate entries
        startupAngleEntry = myTable.getEntry("startupAngle");
        useFakeGyroEntry = myTable.getEntry("useFakeGyro");
        rotationSpeedEntry = myTable.getEntry("rotationSpeed");
        driveSpeedEntry = myTable.getEntry("driveSpeed");
        rotationPidSetpointsEntry = myTable.getEntry("rotationPidSetpoints");
        lateralPidSetpointsEntry = myTable.getEntry("lateralPidSetpoints");
        forwardPidSetpointsEntry = myTable.getEntry("forwardPidSetpoints");
        fieldOrientedEntry = myTable.getEntry("fieldOriented");
        forwardSpeedEntry = myTable.getEntry("forwardSpeed");
        lateralSpeedEntry = myTable.getEntry("lateralSpeed");
        rotationAngleEntry = myTable.getEntry("rotationAngle");
        targetActionPoseEntry = myTable.getEntry("targetActionPose");
        lateralReachedEntry = myTable.getEntry("lateralReached");
        forwardReachedEntry = myTable.getEntry("forwardReached");
        rotationReachedEntry = myTable.getEntry("rotationReached");
        settleCountEntry = myTable.getEntry("settleCount");
        positionerHealthyEntry = myTable.getEntry("positionerHealthy");
        positionHealthReasonEntry = myTable.getEntry("positionHealthReason");
        newAngleRadEntry = myTable.getEntry("newAngleRad");
        positionDeltaEntry = myTable.getEntry("positionDelta");
        positionTargetEntry = myTable.getEntry("positionTarget");
        rotationDeltaEntry = myTable.getEntry("rotationDelta");
        rotationTargetEntry = myTable.getEntry("rotationTarget");
        lookTargetAngEntry = myTable.getEntry("lookTargetAng");
        lookTargetPosEntry = myTable.getEntry("lookTargetPos");
        currentGyroAngleEntry = myTable.getEntry("currentGyroAngle");
        currentPositionEntry = myTable.getEntry("currentPosition");
        fakeAngleEntry = myTable.getEntry("fakeAngle");
        targetDeltaEntry = myTable.getEntry("targetDelta");

        // set initial values
        startupAngleEntry.setDouble(startupAngle);
        useFakeGyroEntry.setBoolean(useFakeGyro);
        rotationSpeedEntry.setDouble(rotationSpeed);
        driveSpeedEntry.setDouble(driveSpeed);
        rotationPidSetpointsEntry.setString(String.format("%f %f %f", rotationPidController.getP(), rotationPidController.getI(), rotationPidController.getD()));
        lateralPidSetpointsEntry.setString(String.format("%f %f %f", lateralPidController.getP(), lateralPidController.getI(), lateralPidController.getD()));
        forwardPidSetpointsEntry.setString(String.format("%f %f %f", forwardPidController.getP(), forwardPidController.getI(), forwardPidController.getD()));
        fieldOrientedEntry.setBoolean(isFieldOriented);

        // for AdvantageScope Visualization
        currentPosePublisher = myTable.getStructTopic("currentPose", Pose3d.struct).publish();
        swerveModuleReqestPublisher = myTable.getStructArrayTopic("requestedModuleStates", SwerveModuleState.struct).publish();
        swerveModuleCommandedPublisher = myTable.getStructArrayTopic("actualModuleStates", SwerveModuleState.struct).publish();
        chassisSpeedsPublisher = myTable.getStructTopic("chassisSpeeds", ChassisSpeeds.struct).publish();

        positioner.Initialize();

        for (SwerveMotorModule module : driveModules) {
            module.Initialize();
        }
    }

    public void SetCurrentPose(Pose3d newPose) {
        SwerveModulePosition[] positions = new SwerveModulePosition[driveModules.length];

        for (SwerveMotorModule module : driveModules) {
            positions[module.GetSwervePosition().getValue()] = module.getPosition();
        }

        poseEstimator.resetPose(newPose.toPose2d());
        //currentAngle = newPose.getRotation().getZ();

        positionInitialized = true;
        myTable.getEntry("positionInitialized").setBoolean(positionInitialized);
    }

    public void SetFieldOriented(boolean value) {
        System.out.printf("SetFieldOriented: %b\n", value);
        if (isFieldOriented != value) {
            isFieldOriented = value;
            fieldOrientedEntry.setBoolean(value);
        }
    }

    public void ToggleFieldOriented(boolean pressed) {
        if (pressed && !wasFieldOrientedPressed)
            SetFieldOriented(!isFieldOriented);

        wasFieldOrientedPressed = pressed;
    }

    public boolean IsFieldOriented() {
        return isFieldOriented;
    }

    // Zero all commanded outputs and optionally push zeros to hardware
    private void zeroDriveCommands() {
        zeroPositionCommands();

        // clear commanded speeds
        ProcessRotationAngle(0.0);

        // reset controllers to avoid residual outputs
        if (rotationPidController != null) rotationPidController.reset();
    }

    // Zero all commanded outputs and optionally push zeros to hardware
    private void zeroPositionCommands() {
        // clear commanded speeds
        ProcessForwardSpeed(0.0);
        ProcessLateralSpeed(0.0);

        // reset controllers to avoid residual outputs
        if (lateralPidController != null) lateralPidController.reset();
        if (forwardPidController != null) forwardPidController.reset();
    }

    public double getGyroAngle() {
        var newAngle = useFakeGyro ? currentAngle
        // modulo the result to get rid of rotation count
        : gyro.getAngle() % 360;

        newAngle -= angleOffset;

        return newAngle;        
    }

    public void ProcessForwardSpeed(double value) {
        forwardSpeed = value;
        wroteForwardThisTick = true; // mark open-loop write this tick
        forwardSpeedEntry.setDouble(forwardSpeed);
    }

    public void ProcessLateralSpeed(double value) {
        lateralSpeed = value;
        wroteLateralThisTick = true; // mark open-loop write this tick
        lateralSpeedEntry.setDouble(lateralSpeed);
    }

    public void ProcessRotationAngle(double value) {
        rotationAngle = value;
        wroteRotationThisTick = true; // mark open-loop write this tick
        rotationAngleEntry.setDouble(rotationAngle);
    }

    public double getInvertedGyroValue() {
        double gyroRaw = getGyroAngle();
        double inverseAngle = ((-gyroRaw % 360.0) + 360.0) % 360.0;

        return Units.degreesToRadians(inverseAngle);  
    }

    public double getCurrentGyroValue() {
        double gyroRaw = getGyroAngle();
        double newAngle = ((gyroRaw % 360.0) + 360.0) % 360.0;

        return Units.degreesToRadians(newAngle);  
    }

    public void StopRotation() {
        ProcessRotationAngle(0.0);
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

    public ActionPose GetActionPose(Group group, int location, int locationIndex, int position, Action action) {
        for (ActionPose pose : actionPoses) {
            if (
                (pose.group == group || pose.group == Group.Any)
                && (pose.locationIndex == locationIndex || pose.locationIndex == -1)
                && (pose.location == location || pose.location == -1)
                && (pose.position == position || pose.position == -1)
                && (pose.action == action || pose.action == Action.Any)
            ) {
                System.out.printf("%s GetActionPose: Matched %s %d %d %d %s\n", moduleID, pose.group, pose.location, pose.locationIndex, pose.position, pose.action);
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
    
    public void SetTargetActionPose(Group group, int location, int locationIndex, int position, Action action) {
        ActionPose actionPose = GetActionPose(group, location, locationIndex, position, action);

        if (actionPose != null) {
            targetPose = actionPose;
            targetActionPoseEntry.setString(String.format("%s %s %d %s %s", group, location, locationIndex, position, action));
            lateralReachedEntry.setBoolean(false);
            forwardReachedEntry.setBoolean(false);
            rotationReachedEntry.setBoolean(false);
            
            // reset settle counter on new target
            settleCount = 0;
            settleCountEntry.setNumber(settleCount);
        }
    }

    public boolean isPositionerHealthy() {
        var healthy = true;

        if (Robot.isReal())
            healthy = positioner.IsHealthy();

        positionerHealthyEntry.setBoolean(healthy);
        positionHealthReasonEntry.setString(positioner.GetHealthReason());

        return healthy;
    }

    public void EvaluateTargetPose(Pose3d currentPose, double newAngleRad) {
        if (targetPose != null) {
            newAngleRadEntry.setDouble(newAngleRad);
            var pose = targetPose.target;
            var targetPosition = pose.Position;
            var targetRotation = pose.Orientation;

            var lateralReached = false;
            var forwardReached = false;
            var rotationReached = false;
            var positionerHealthy = isPositionerHealthy();
            var posNorm = currentPosition.getNorm();

            // TODO 1: re-evaluate if this is needed with estimator
            var seekTag = false; // posNorm == 0.0;
            myTable.getEntry("seekTag").setBoolean(seekTag);

            if (!seekTag) {
                seekRotation = 0.0;
                seekStart = 0.0;
                seekReversed = false;
            }

            rotationReachedEntry.setBoolean(rotationReached);
            lateralReachedEntry.setBoolean(lateralReached);
            forwardReachedEntry.setBoolean(forwardReached);

            // only process position if we have a target
            if (pose.HasPosition) {
                var positionDelta = targetPosition.minus(currentPosition);
                var isTravelGroup = targetPose.group == Group.Travel;
                if (isTravelGroup) {
                    var positionTolerance = frameNorm;
                    myTable.getEntry("_deltaNorm").setDouble(positionDelta.getNorm());
                    myTable.getEntry("_positionTolerance").setDouble(positionTolerance);
                    if (positionDelta.getNorm() < positionTolerance) {
                        // travel group we don't want to stop on position, but find next waypoint when we get close
                        lateralReached = true;
                        forwardReached = true;
                    }
                } else {
                    myTable.getEntry("_deltaNorm").unpublish();
                    myTable.getEntry("_positionTolerance").unpublish();
                }

                if (!lateralReached || !forwardReached) {
                    positionDeltaEntry.setString(positionDelta.toString());
                    positionTargetEntry.setString(targetPosition.toString());
                    myTable.getEntry("posNorm").setDouble(posNorm);

                    if (posNorm > 0.0) { // requires position to be initialized
                        // limelight team-based origin is x forward positive, y left positive - same as FRC field
                        // TODO: figure out how to allow target evaluation in teleop - game controller sends values for position constantly and overrides this, I think?
                        if (!wroteLateralThisTick) { // allows game controller precedence
                            var lateralSpeed = lateralPidController.calculate(currentPosition.getY(), targetPosition.getY());
                            // clamp to real values
                            lateralSpeed = Math.max(-driveSpeed, Math.min(driveSpeed, lateralSpeed));
                            if (Math.abs(lateralSpeed) < floatTolerance) {
                                lateralReached = true;
                                lateralReachedEntry.setBoolean(lateralReached);
                                ProcessLateralSpeed(0.0);
                            } else if (positionerHealthy) { // prevents sending wrong coordinates
                                ProcessLateralSpeed(lateralSpeed);
                            }
                        }

                        if (!wroteForwardThisTick) { // allows game controller precedence
                            var forwardSpeed = forwardPidController.calculate(currentPosition.getX(), targetPosition.getX());
                            // clamp to real values
                            forwardSpeed = Math.max(-driveSpeed, Math.min(driveSpeed, forwardSpeed));
                            if (Math.abs(forwardSpeed) < floatTolerance) {
                                forwardReached = true;
                                forwardReachedEntry.setBoolean(forwardReached);
                                ProcessForwardSpeed(0.0);
                            } else if (positionerHealthy) { // prevents sending wrong coordinates
                                ProcessForwardSpeed(forwardSpeed);
                            }
                        }
                    } else if (!wroteLateralThisTick && !wroteForwardThisTick) {
                        // shut down any previous drive commands because we lost our position
                        // TODO: need to smooth this out somehow, but keep it safe - jitters with limelight
                        zeroPositionCommands();
                    }
                }
            } else {
                // avoid deadlock, if we don't have a position defined, short circuit
                lateralReached = true;
                forwardReached = true;
            }

            // TODO: why isn't this called consistently?
            // if we have a lookat and we've reached our position, don't keep trying to find the lookat
            myTable.getEntry("hasLookAt").setBoolean(pose.HasLookAt);
            if (pose.HasLookAt && lateralReached && forwardReached)
                rotationReached = true;

            // only process rotation if we have a target and haven't already been overridden this tick
            if (!rotationReached && (pose.HasOrientation || pose.HasLookAt || seekTag)) {
                if (!wroteRotationThisTick) { // allows game controller precedence
                    double targetValue = 0.0;
                    boolean processAngle = false;

                    if (pose.HasOrientation) {
                        // TODO 1: is this still a problem? might have been fixed with auto selector fixes; this doesn't seem to work if we're on the other side of zero?
                        var targetRad = (targetRotation.getRadians() + (2 * Math.PI)) % (2 * Math.PI); // wrap to positive angles
                        var rotationDelta = targetRad - newAngleRad;
                        rotationDeltaEntry.setDouble(rotationDelta);
                        rotationTargetEntry.setDouble(targetRotation.getRadians());
                        targetValue = targetRotation.getRadians();
                        processAngle = true;
                    } else if (pose.HasLookAt && posNorm > 0.0) {
                        // trying to rotate from the camera's vantage point to ensure that the april tags are always in the center
                        var adjustedPos = Robot.isReal() ? positioner.GetReferenceInFieldCoords() : fakeCameraPose.transformBy(currentPose.minus(fakeCameraPose));
                        myTable.getEntry("adjustedPos").setString(adjustedPos.toString());
                        var lookAt = pose.LookAt;
                        // currently, this gets the atan with axes flipped and then subtracts from negative field orientation
                        // var lookTarget = -1.566 - Math.atan2(lookAt.getX() - adjustedPos.getX(), lookAt.getY() - adjustedPos.getY());
                        var lookTarget = Math.atan2(lookAt.getY() - adjustedPos.getY(), lookAt.getX() - adjustedPos.getX());
                        myTable.getEntry("lookTargetRaw").setDouble(lookTarget);
                        // adjust rotation for camera rotation offset
                        // TODO 1: make sure limelight rotation is positive for real
                        var referenceAngle = Robot.isReal() ? positioner.GetReferenceInRobotCoords().getRotation().getZ() : fakeCameraPose.getRotation().getZ();
                        lookTarget = lookTarget - referenceAngle;
                        myTable.getEntry("lookTargetAdj").setDouble(lookTarget);
                        // wrap to positive and modulo
                        lookTarget = (lookTarget + (2 * Math.PI)) % (2 * Math.PI);
                        lookTargetAngEntry.setDouble(lookTarget);
                        lookTargetPosEntry.setString(lookAt.toString());
                        targetValue = lookTarget;
                        processAngle = true;
                    }

                    if (processAngle) { // only try to process the angle if we've given it one
                        // TODO 1: make sure we're rotating the right direction for real - positive values here result in negative navx values, I think.
                        var rotationSpeed = rotationPidController.calculate(newAngleRad, targetValue);
                        // TODO 1: evaluate if this works as expected - may need to adjust the PID controller for all rotations?
                        if (pose.HasLookAt)
                            rotationSpeed *= 1.5; // boost lookat rotation speed a bit
                        // clamp to real values
                        rotationSpeed = Math.max(-this.rotationSpeed, Math.min(this.rotationSpeed, rotationSpeed));
                        // check angle by rotation to avoid mismatched signs and other things from preventing target matching
                        if (Math.abs(new Rotation2d(newAngleRad).minus(new Rotation2d(targetValue)).getRadians()) < floatTolerance) {
                            // only mark 'reached' if we don't have a lookat target or our position is also reached
                            if (!pose.HasLookAt || (lateralReached && forwardReached))
                                rotationReached = true;
                            rotationReachedEntry.setBoolean(rotationReached);
                            ProcessRotationAngle(0.0);
                        } else {
                            ProcessRotationAngle(rotationSpeed);
                        }
                    } else if(seekTag && DriverStation.isAutonomous()) {
                        if (seekRotation == 0.0) {
                            double sign = previousRotationSpeed >= 0.0 ? -1.0 : 1.0;
                            seekRotation = sign * 0.25;
                            seekStart = newAngleRad;
                            seekReversed = false;
                        }

                        if (!seekReversed && Math.abs(seekStart - newAngleRad) > 1.57) {
                            // if we've turned 90 degrees from our starting point, flip direction
                            seekRotation = -seekRotation;
                            seekReversed = true;
                        }

                        // slowly turn the opposite direction until we find our position
                        ProcessRotationAngle(seekRotation);
                    }
                }
            } else {
                // avoid deadlock, if we don't have a rotation defined, short circuit
                rotationReached = true;
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
            settleCountEntry.setNumber(settleCount);
        }
    }

    public void AbandonTarget() {
        targetPose = null;

        zeroDriveCommands();
        
        targetActionPoseEntry.setString("none");
        lateralReachedEntry.unpublish();
        forwardReachedEntry.unpublish();
        rotationReachedEntry.unpublish();
        targetDeltaEntry.unpublish();
        rotationDeltaEntry.unpublish();
    }

    public void ProcessState(boolean isAuto) {
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.htm
        // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
        // https://www.chiefdelphi.com/t/set-motor-position-with-encoder/152088/3

        // Invert the Gyro angle because it rotates opposite of the robot steering, then wrap it to a positive value
        // TODO 1: check this after adjusting coordinate systems
        double currentGyroAngle = getInvertedGyroValue();
        currentGyroAngleEntry.setDouble(currentGyroAngle);

        if (Robot.isReal()) {
            PoseEstimate visionEstimate = positioner.GetPoseEstimate();
            poseEstimator.addVisionMeasurement(visionEstimate.pose, visionEstimate.latency);

            if (!positionInitialized && visionEstimate.pose.getTranslation().getNorm() > 0.0) {
                positionInitialized = true;
                myTable.getEntry("positionInitialized").setBoolean(positionInitialized);

                poseEstimator.resetPose(visionEstimate.pose);
            }
        }

        Pose3d currentPose = new Pose3d(poseEstimator.getEstimatedPosition());
        currentPosePublisher.set(currentPose);

        // update simulator field position
        fieldPosition.setRobotPose(currentPose.toPose2d());
        SmartDashboard.putData("Field", fieldPosition);

        // TODO 1: Identify if this is correct - does it need inverse or does everything use normal?
        // yaw is in degrees
        var limelightAngle = currentGyroAngle / ((Math.PI * 2) / 360);
        myTable.getEntry("limelightAngleRaw").setDouble(limelightAngle);
        myTable.getEntry("limelightAngleRadAdj").setDouble((limelightAngle + 180) % 360);
        positioner.SetRobotOrientation("", limelightAngle, 0,0,0,0,0);

        currentPosition = currentPose.getTranslation();

        EvaluateTargetPose(currentPose, currentGyroAngle);

        // prevents bleedover between target poses
        if (isAuto && targetPose == null
            && !(wroteForwardThisTick || wroteLateralThisTick || wroteRotationThisTick))
            // if we're in auto and have no target, zero all outputs
            zeroDriveCommands();

        // prevents command bleedover from previous ticks
        if (targetPose != null && !isPositionerHealthy()
            && !(wroteForwardThisTick || wroteLateralThisTick))
            // if our position is invalid, zero position outputs
            zeroPositionCommands();

        // set the chassis speed object according to current controller values
        // modifiers only affect open-loop values, not Auto values
        double forwardSpeed = this.forwardSpeed * controller.ApplyModifiers(driveSpeed);
        double lateralSpeed = this.lateralSpeed * controller.ApplyModifiers(driveSpeed);
        double thisRotationSpeed = rotationAngle * controller.ApplyModifiers(rotationMultiplier); //rotationAngle; // * controller.ApplyModifiers(this.rotationSpeed);
        rotationSpeedEntry.setDouble(thisRotationSpeed);

        var currentRotation = Rotation2d.fromRadians(currentGyroAngle);

        ChassisSpeeds speeds = isFieldOriented ?
            ChassisSpeeds.fromFieldRelativeSpeeds(
                // invert directions if we're red and manually controlling
                Utility.IsRedAlliance() && DriverStation.isTeleop() ? -forwardSpeed : forwardSpeed, // negate for red teleop (field rotation)
                Utility.IsRedAlliance() && DriverStation.isTeleop() ? -lateralSpeed : lateralSpeed, // negate for red teleop (field rotation)
                thisRotationSpeed, currentRotation)
            : new ChassisSpeeds(forwardSpeed,
                lateralSpeed,
                //Utility.IsRedAlliance() && DriverStation.isTeleop() ? -lateralSpeed : lateralSpeed, // negate for red teleop (field rotation)
                thisRotationSpeed);
        chassisSpeedsPublisher.set(speeds);

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

        SwerveModulePosition[] positions = new SwerveModulePosition[driveModules.length];
        double[] driveSpeeds = new double[4]; //always 4 because of dashboard setup

        SwerveModuleState[] actualStates = new SwerveModuleState[driveModules.length];
        for (SwerveMotorModule module : driveModules) {
            var i = module.GetSwervePosition().getValue();
            moduleStates[i] = module.updateModuleValues(moduleStates[i], optimize);
            actualStates[i] = module.getCurrentState();
            positions[i] = module.getPosition();
            if (i < driveSpeeds.length)
                driveSpeeds[i] = module.getSpeed();
        }
        swerveModuleReqestPublisher.set(moduleStates);
        swerveModuleCommandedPublisher.set(actualStates);

        odometry.update(currentRotation, positions);
        poseEstimator.update(currentRotation, positions);

        var primaryModule = driveModules[0];
        if (primaryModule != null) {
            // calculate and store current field position and rotation
            // var centerOffset = new Translation2d(Math.cos(getGyroAngle()) * primaryModule.modulePosition.getX(),
            //         Math.sin(getGyroAngle()) * primaryModule.modulePosition.getY());
            currentPositionEntry.setString(currentPosition.toString());
            if ((Math.abs(previousPosition.minus(currentPosition).getX()) > floatTolerance
                    || Math.abs(previousPosition.minus(currentPosition).getY()) > floatTolerance)
                    && Math.abs(previousPosition.minus(currentPosition).getNorm()) < deltaLimit) {
                previousPosition = currentPosition;
            }
        }

        // update fake gyro angle
        currentAngle += -thisRotationSpeed * fakeGyroRate;
        fakeAngleEntry.setDouble(currentAngle);
        previousAngle = currentAngle;
        previousRotationSpeed = thisRotationSpeed;

        // update dashboard
        SmartDashboard.putNumberArray("RobotDrive Motors", new double[] {driveModules[0].getSpeed(), driveModules[1].getSpeed(), 0.0, 0.0});
        //SmartDashboard.putNumberArray("My Motors", new double[] {driveModules.get(0).getSpeed(), driveModules.get(1).getSpeed(), 0.0, 0.0});
        //System.out.printf("leftFront speed: %f\n", driveModules.get(0).getSpeed()); // , driveModules.get(1).getSpeed(), 0.0, 0.0});

        wroteForwardThisTick = false;
        wroteLateralThisTick = false;
        wroteRotationThisTick = false;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    @Override
    public Pose3d GetPosition() {
        return isPositionerHealthy() ? positioner.GetPose() : new Pose3d(positioner.GetPose().getTranslation(), new Rotation3d(0, 0, getCurrentGyroValue()));
        // return new Pose3d(newPosition, new Rotation3d(0, 0, getCurrentGyroValue()));
        // return new Translation3d(currentPosition.getX(), currentPosition.getY(), currentAngle);
    }
}
