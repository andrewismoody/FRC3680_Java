package frc.robot.modules;

import java.util.ArrayList;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.action.Group;
import frc.robot.encoder.Encoder;
import frc.robot.gyro.Gyro;
import frc.robot.misc.Utility;
import frc.robot.positioner.LimelightHelpers.PoseEstimate;
import frc.robot.positioner.Positioner;
import frc.robot.z2024.Constants;
import frc.robot.action.Action;
import frc.robot.action.ActionPose;

public class DifferentialDriveModule implements DriveModule {
    String moduleID;

    ModuleController controller;
    DifferentialDrive driveController;
    MotorController leftMotor;
    MotorController rightMotor;
    Gyro gyro;
    Positioner positioner;
    Pose3d fakeCameraPose = new Pose3d(new Translation3d(0, -0.28, 0.51), new Rotation3d(new Rotation2d(90)));

    double forwardSpeed = 0.0;
    double rotationAngle = 0.0;

    public boolean debugAngle = false;
    public boolean debugSpeed = false;

    double currentForwardSpeed = 0.0;
    double currentRotationAngle = 0.0;
    double previousForwardSpeed = 0.0;
    double previousRotationAngle = 0.0;

    boolean isFieldOriented = false;
    boolean wasFieldOrientedPressed = false;

    Translation3d currentPosition = Translation3d.kZero;

    ArrayList<ActionPose> actionPoses = new ArrayList<ActionPose>();
    ActionPose targetPose;

    DifferentialDrivePoseEstimator poseEstimator;
    DifferentialDriveKinematics kinematics;
    Pose3d currentPose = Pose3d.kZero;
    Field2d fieldPosition;

    double angleOffset = 0.0;
    double leftDistance = 0.0;
    double rightDistance = 0.0;
    double floatTolerance;
    double fakeGyroRate = 0.06; // how fast the robot can rotate in radians per timeslice (0.02s)
    double elapsedTime = 0.0;
    double previousTime = 0.0;
    double driveRatio = 1.0; // wheelCircumference / gearRatio
    double driveEncSimRate = 12.0;
    double frameNorm = 0.0;
    double driveSpeed;
   
    PIDController forwardPidController;
    PIDController rotationPidController;

    // Single settle counter: require N consecutive cycles with all axes reached
    private int settleCyclesRequired = 3; // tune as needed
    private int settleCount = 0;

    private boolean wroteForwardThisTick = false;
    private boolean wroteRotationThisTick = false;
    boolean startLookatAchieved = false;

    Encoder leftDriveEncoder;
    Encoder rightDriveEncoder;

    NetworkTable myTable;
    private NetworkTableEntry startupAngleEntry;
    private NetworkTableEntry elapsedTimeEntry;
    // private NetworkTableEntry rotationSpeedEntry;
    // private NetworkTableEntry driveSpeedEntry;
    private NetworkTableEntry rotationPidSetpointsEntry;
    private NetworkTableEntry forwardPidSetpointsEntry;
    private NetworkTableEntry fieldOrientedEntry;
    private NetworkTableEntry targetActionPoseEntry;
    private NetworkTableEntry currentGyroAngleEntry;
    StructPublisher<Pose3d> currentPosePublisher;
    StructPublisher<Pose3d> targetPosePublisher;
    StructPublisher<Pose3d> transTargetPosePublisher;
    StructPublisher<Pose3d> lookatPosePublisher;

    public DifferentialDriveModule(String ModuleID, Gyro Gyro, Positioner Positioner, double DriveSpeed, MotorController LeftMotor, Encoder LeftDriveEncoder, MotorController RightMotor, Encoder RightDriveEncoder, double DriveRatio, double FloatTolerance, double FrameNorm) {
        moduleID = ModuleID;
        leftMotor = LeftMotor;
        rightMotor = RightMotor;
        positioner = Positioner;
        gyro = Gyro;
        driveRatio = DriveRatio;
        floatTolerance = FloatTolerance;
        driveSpeed = DriveSpeed;

        leftDriveEncoder = LeftDriveEncoder;
        rightDriveEncoder = RightDriveEncoder;

        frameNorm = FrameNorm;

        myTable = NetworkTableInstance.getDefault().getTable(moduleID);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        if (rightMotor instanceof SparkMax)
            ((SparkMax)rightMotor).configure(new SparkMaxConfig().inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        else
            rightMotor.setInverted(true);

        driveController = new DifferentialDrive(leftMotor::set, rightMotor::set);
        driveController.setSafetyEnabled(false);

        kinematics = new DifferentialDriveKinematics(Constants.robotSize.getY());
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(getGyroRadians()), leftDistance, rightDistance, Pose2d.kZero);
        fieldPosition = new Field2d();
        
        var posKp = 2.0; 
        var posKi = 0; //posKp * 0.10;
        var posKd = 0; //posKi * 3.0;
        forwardPidController = new PIDController(posKp, posKi, posKd);

        // bumping this up to 1.0 after adjusting the rotation multiplier from processrotation
        var rotKp = 1.0;
        var rotKi = 0; // rotKp * 0.10;
        var rotKd = 0; //rotKi * 3.0;
        rotationPidController = new PIDController(rotKp, rotKi, rotKd);
        rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        rotationPidController.setTolerance(floatTolerance);
    }

    public void ResetGyro() {
        gyro.reset();
    }

    public double getGyroAngle() {
        // typically returned in degrees
        var newAngle = gyro.getAngle();

        newAngle -= angleOffset;

        return newAngle;        
    }

    public double getGyroRadians() {
        double gyroRaw = getGyroAngle();

        return Units.degreesToRadians(gyroRaw);  
    }

    public void ResetEncoders() {
        // not implemented
    }

    public String GetModuleID() {
        return moduleID;
    }

    public void AddActionPose(ActionPose newAction) {
        if (GetActionPose(newAction.group, newAction.location, newAction.locationIndex, newAction.position, newAction.action) == null) {
            actionPoses.add(newAction);
        }
    }

    @Override
    public void ApplyInverse(boolean isAuto) {
        // not implemented
    }

    @Override
    public void ApplyValue(boolean isAuto) {
        // not implemented
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

        // reset controllers to avoid residual outputs
        if (forwardPidController != null) forwardPidController.reset();
    }

    public boolean isPositionerHealthy() {
        var healthy = true;

        if (RobotBase.isReal())
            healthy = positioner.IsHealthy();

        return healthy;
    }

    public void EvaluateTargetPose(Pose3d currentPose, double newAngleRad) {
        if (targetPose != null) {
            var pose = targetPose.target;
            var targetPosition = pose.Position;
            var targetRotation = pose.Orientation;

            var forwardReached = false;
            var rotationReached = false;
            var positionerHealthy = isPositionerHealthy();
            var posNorm = currentPosition.getNorm();

            Translation3d transformedTarget;
            Rotation2d heading = Rotation2d.kZero;
            boolean hasHeading = false;

            // only process position if we have a target
            if (pose.HasPosition) {
                var positionDelta = targetPosition.minus(currentPosition);
                // for differential drive, we need to rotate the field coordinates into robot coordinates and then find whether we're going forward or backward to reach the target
                transformedTarget = targetPosition.rotateAround(currentPose.getTranslation(), currentPose.getRotation().unaryMinus());
                transTargetPosePublisher.set(new Pose3d(transformedTarget, currentPose.getRotation()));
                // get the heading that is closest to our current angle - we'll back up to the target if needed
                var forwardHeading = Utility.getLookat(currentPose.getTranslation().toTranslation2d(), targetPosition.toTranslation2d());
                var reverseHeading = Utility.getLookat(currentPose.getTranslation().toTranslation2d(), targetPosition.toTranslation2d()).plus(Rotation2d.k180deg);
                if (Math.abs(forwardHeading.minus(currentPose.getRotation().toRotation2d()).getRadians()) > Math.abs(reverseHeading.minus(currentPose.getRotation().toRotation2d()).getRadians())) {
                    heading = reverseHeading;
                } else {
                    heading = forwardHeading;
                }
                hasHeading = true;

                if (Utility.isTravelGroup(targetPose.group)) {
                    var positionTolerance = frameNorm / 2;
                    if (positionDelta.getNorm() < positionTolerance) {
                        // travel group with lookat rotation we don't want to stop on position, but find next waypoint when we get close
                        if (pose.HasLookAt)
                            // other rotation modes may continue to change angle after this.
                            System.out.printf("%d ms: lookAt travelGroup tolerance reached\n", System.currentTimeMillis());
                        forwardReached = true;
                        settleCount = settleCyclesRequired; // force settle
                    }
                }

                if (!forwardReached) {
                    if (posNorm > 0.0 && (!pose.HasLookAt || startLookatAchieved)) { // requires position to be initialized
                        // limelight team-based origin is x forward positive, y left positive - same as FRC field
                        // TODO: figure out how to allow target evaluation in teleop - game controller sends values for position constantly and overrides this, I think?
                        if (!wroteForwardThisTick) { // allows game controller precedence
                            var forwardSpeed = forwardPidController.calculate(currentPosition.getX(), transformedTarget.getX());
                            // clamp to real values
                            forwardSpeed = Math.max(-driveSpeed, Math.min(driveSpeed, forwardSpeed));
                            if (Math.abs(forwardSpeed) < floatTolerance) {
                                forwardReached = true;
                                ProcessForwardSpeed(0.0);
                            } else if (positionerHealthy) { // prevents sending wrong coordinates
                                ProcessForwardSpeed(forwardSpeed);
                            }
                        }
                    } else if (!wroteForwardThisTick) {
                        // shut down any previous drive commands because we lost our position
                        // TODO: need to smooth this out somehow, but keep it safe - jitters with limelight
                        zeroPositionCommands();
                    }
                }
            } else {
                // avoid deadlock, if we don't have a position defined, short circuit
                forwardReached = true;
            }

            double lookTarget = 0.0;
            if (pose.HasLookAt) {
                // if we have a lookat and we've reached our position, don't keep trying to find the lookat
                if (forwardReached) {
                    System.out.printf("%d ms: lookat position reached\n", System.currentTimeMillis());
                    rotationReached = true;
                }

                // trying to rotate from the camera's vantage point to ensure that the april tags are always in the center
                var adjustedPos = GetPositionerOffset().getTranslation();
                var referenceAngle = GetPositionerOffset().getRotation().getZ();
                lookTarget = Utility.getLookat(adjustedPos.toTranslation2d(), pose.LookAt.toTranslation2d()).getRadians();
                // adjust rotation for camera rotation offset
                lookTarget = lookTarget - referenceAngle;
                // wrap to positive and modulo
                lookTarget = (lookTarget + (2 * Math.PI)) % (2 * Math.PI);
                lookatPosePublisher.set(new Pose3d(pose.LookAt, new Rotation3d(new Rotation2d(lookTarget))));
            }

            // only process rotation if we have a target and haven't already been overridden this tick
            if (!rotationReached && (pose.HasOrientation || pose.HasLookAt || (!forwardReached && !heading.equals(Rotation2d.kZero)))) {
                if (!wroteRotationThisTick) { // allows game controller precedence
                    double targetValue = 0.0;
                    boolean processAngle = false;

                    if (!forwardReached && hasHeading && (!pose.HasLookAt || startLookatAchieved)) {
                        targetValue = heading.getRadians();
                        processAngle = true;
                    } else if (pose.HasOrientation) {
                        targetValue = targetRotation.getRadians();
                        processAngle = true;
                    } else if (pose.HasLookAt && posNorm > 0.0) {
                        // TODO 1: make sure limelight rotation is positive for real
                        targetValue = lookTarget;
                        processAngle = true;
                    }

                    if (processAngle) { // only try to process the angle if we've given it one
                        var rotationSpeed = rotationPidController.calculate(newAngleRad, targetValue);
                        // TODO 1: evaluate if this works as expected - may need to adjust the PID controller for all rotations?
                        if (pose.HasLookAt)
                            rotationSpeed *= 1.5; // boost lookat rotation speed a bit
                        // clamp to real values
                        // invert rotation for auto - WHY??
                        rotationSpeed = -Math.max(-driveSpeed, Math.min(this.driveSpeed, rotationSpeed));
                        // check angle by rotation to avoid mismatched signs and other things from preventing target matching
                        if (Math.abs(new Rotation2d(newAngleRad).minus(new Rotation2d(targetValue)).getRadians()) < floatTolerance) {
                            // only mark 'reached' if we don't have a lookat target or our position is also reached
                            if (!pose.HasLookAt || forwardReached)
                                rotationReached = true;
                            if (pose.HasLookAt && !startLookatAchieved) {
                                System.out.println("startLookatAchieved");
                                startLookatAchieved = true;
                            }
                            ProcessRotationAngle(0.0);
                        } else {
                            ProcessRotationAngle(rotationSpeed);
                        }
                    }
                }
            } else {
                // avoid deadlock, if we don't have a rotation defined, short circuit
                rotationReached = true;
            }

            if (forwardReached && rotationReached) {
                System.out.printf("%d ms: all motion targets reached\n", System.currentTimeMillis());
                // increment global settle counter; do not abandon until threshold reached
                if (settleCount < settleCyclesRequired) {
                    settleCount++;
                }
                if (settleCount >= settleCyclesRequired) {
                    System.out.printf("%d ms: settlecount reached\n", System.currentTimeMillis());
                    AbandonTarget();
                }
            } else {
                // any axis out of tolerance resets the settle window
                settleCount = 0;
            }
        }
    }

    public void ProcessState(boolean isAuto) {        
        var now = System.currentTimeMillis();

        if (previousTime == 0) {
            elapsedTime = 0;
        } else {
        elapsedTime = now - previousTime;
        }
        previousTime = now;
        elapsedTimeEntry.setDouble(elapsedTime);

        double currentGyroAngle = getGyroRadians();
        currentGyroAngleEntry.setDouble(currentGyroAngle);

        var limelightAngle = Utility.radiansToDegrees(currentGyroAngle);
        myTable.getEntry("limelightAngleRaw").setDouble(limelightAngle);
        positioner.SetRobotOrientation("", limelightAngle, 0,0,0,0,0);

        if (RobotBase.isReal()) {
            PoseEstimate visionEstimate = positioner.GetPoseEstimate();
            if (visionEstimate != null)
                poseEstimator.addVisionMeasurement(visionEstimate.pose, visionEstimate.latency);
        }

        currentPose = new Pose3d(poseEstimator.getEstimatedPosition());
        currentPosePublisher.set(currentPose);

        // update simulator field position
        fieldPosition.setRobotPose(currentPose.toPose2d());
        SmartDashboard.putData("Field", fieldPosition);

        currentPosition = currentPose.getTranslation();
        var currentRotation = Rotation2d.fromRadians(currentGyroAngle);

        EvaluateTargetPose(currentPose, currentGyroAngle);
        
        // prevents bleedover between target poses
        if (isAuto && targetPose == null
            && !(wroteForwardThisTick || wroteRotationThisTick))
            // if we're in auto and have no target, zero all outputs
            zeroDriveCommands();

        // prevents command bleedover from previous ticks
        if (targetPose != null && !isPositionerHealthy()
            && !wroteForwardThisTick)
            // if our position is invalid, zero position outputs
            zeroPositionCommands();

        currentForwardSpeed = controller.ApplyModifiers(forwardSpeed);
        currentRotationAngle = controller.ApplyModifiers(rotationAngle);

        WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(currentForwardSpeed, currentRotationAngle, true);
        myTable.getEntry("leftSpeed").setDouble(speeds.left);
        myTable.getEntry("rightSpeed").setDouble(speeds.right);

        leftMotor.set(speeds.left);
        rightMotor.set(speeds.right);
       
        var leftDistanceAccumulation = speeds.left * driveEncSimRate;
        var rightDistanceAccumulation = speeds.right * driveEncSimRate;
        if (!RobotBase.isReal()) {
            if (leftDriveEncoder != null) 
                // fake adjust current distance to simulate encoder input
                leftDriveEncoder.appendSimValueRad(leftDistanceAccumulation);
            if (rightDriveEncoder != null)
                // fake adjust current distance to simulate encoder input
                rightDriveEncoder.appendSimValueRad(rightDistanceAccumulation);
        }

        leftDistance = leftDriveEncoder == null ?
            leftDistance + leftDistanceAccumulation :
            leftDriveEncoder.getRawValue() * driveRatio; // driveRatio is wheelCircumference / gearRatio

        rightDistance = rightDriveEncoder == null ?
            rightDistance + rightDistanceAccumulation :
            rightDriveEncoder.getRawValue() * driveRatio; // driveRatio is wheelCircumference / gearRatio
  
        var positions = new DifferentialDriveWheelPositions(leftDistance, rightDistance);
        //odometry.update(currentRotation, positions);
        poseEstimator.update(currentRotation, positions);

        

        // update fake gyro angle - will append the amount of change from this period to the current value
        gyro.appendSimValueDeg(Utility.radiansToDegrees(currentRotationAngle * fakeGyroRate));

        // update dashboard
        SmartDashboard.putNumberArray("RobotDrive Motors", new double[]{leftMotor.get(), rightMotor.get()});
    }

    public void Initialize() {
        var startupAngle = getGyroAngle();

        currentGyroAngleEntry = myTable.getEntry("currentGyroAngle");
        startupAngleEntry = myTable.getEntry("startupAngle");
        elapsedTimeEntry = myTable.getEntry("elapsedTime");
        // rotationSpeedEntry = myTable.getEntry("rotationSpeed");
        // driveSpeedEntry = myTable.getEntry("driveSpeed");
        rotationPidSetpointsEntry = myTable.getEntry("rotationPidSetpoints");
        forwardPidSetpointsEntry = myTable.getEntry("forwardPidSetpoints");
        fieldOrientedEntry = myTable.getEntry("fieldOriented");
        targetActionPoseEntry = myTable.getEntry("targetActionPose");

        startupAngleEntry.setDouble(startupAngle);
        // rotationSpeedEntry.setDouble(rotationSpeed);
        // driveSpeedEntry.setDouble(driveSpeed);
        rotationPidSetpointsEntry.setString(String.format("%f %f %f", rotationPidController.getP(), rotationPidController.getI(), rotationPidController.getD()));
        forwardPidSetpointsEntry.setString(String.format("%f %f %f", forwardPidController.getP(), forwardPidController.getI(), forwardPidController.getD()));
        fieldOrientedEntry.setBoolean(isFieldOriented);
        targetActionPoseEntry.setString("none");

        // for AdvantageScope Visualization
        currentPosePublisher = myTable.getStructTopic("currentPose", Pose3d.struct).publish();
        targetPosePublisher = myTable.getStructTopic("targetPose", Pose3d.struct).publish();
        transTargetPosePublisher = myTable.getStructTopic("transformedTargetPose", Pose3d.struct).publish();
        lookatPosePublisher = myTable.getStructTopic("lookatPose", Pose3d.struct).publish();

        positioner.Initialize();
    }

    public Pose3d GetPositionerOffset() {
        var adjustedPos = RobotBase.isReal() ? positioner.GetReferenceInFieldCoords() : fakeCameraPose.transformBy(GetPosition().minus(fakeCameraPose));
        var referenceAngle = RobotBase.isReal() ? positioner.GetReferenceInRobotCoords().getRotation().getZ() : fakeCameraPose.getRotation().getZ();
        return new Pose3d(adjustedPos.getTranslation(), new Rotation3d(new Rotation2d(referenceAngle)));
    }

    public void SetCurrentPose(Pose3d newPose) {
        var positions = new DifferentialDriveWheelPositions(leftDistance, rightDistance);

        poseEstimator.resetPosition(newPose.getRotation().toRotation2d(), positions, newPose.toPose2d());
    }

    public void ProcessForwardSpeed(double value) {
        forwardSpeed = value;
    }
  
    public void ProcessLateralSpeed(double value) {
        // Not implemented for differential drive
    }
  
    public void ProcessRotationAngle(double value) {
        rotationAngle = value;
    }

    public void StopRotation() {
        rotationAngle = 0;
    }

    public void SetController(ModuleController Controller) {
        controller = Controller;
    }

    public Pose3d GetPosition() {
        return new Pose3d(currentPosition, new Rotation3d(0, 0, currentRotationAngle));
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
                System.out.printf("%s GetActionPose: Matched %s %d %d %d %s\n", moduleID, group, location, locationIndex, position, action);
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
            if (targetPose.target.HasPosition) {
                if (targetPose.target.HasOrientation)
                    targetPosePublisher.set(new Pose3d(targetPose.target.Position, new Rotation3d(targetPose.target.Orientation)));
                else
                    targetPosePublisher.set(new Pose3d(targetPose.target.Position, Rotation3d.kZero));
            } else
                targetPosePublisher.set(null); // empty pose

            // reset settle counter on new target
            settleCount = 0;
            startLookatAchieved = false;
        }
    }

   public void SetFieldOriented(boolean value) {
        isFieldOriented = value;
        myTable.getEntry("fieldOriented").setBoolean(value);
   }

   public void ToggleFieldOriented(boolean pressed) {
        if (pressed && !wasFieldOrientedPressed)
            SetFieldOriented(!isFieldOriented);

        wasFieldOrientedPressed = pressed;
   }

   public boolean IsFieldOriented() {
       return this.isFieldOriented;
   }

    public void AbandonTarget() {
        targetPose = null;
        targetActionPoseEntry.setValue("none");
    }
}
