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
    Pose3d fakeCameraPose = new Pose3d(new Translation3d(0, -0.28, 0.51), new Rotation3d(new Rotation2d(0)));

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
   
    PIDController forwardPidController;
    PIDController rotationPidController;

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

    public DifferentialDriveModule(String ModuleID, Gyro Gyro, Positioner Positioner, MotorController LeftMotor, Encoder LeftDriveEncoder, MotorController RightMotor, Encoder RightDriveEncoder, double DriveRatio, double FloatTolerance) {
        moduleID = ModuleID;
        leftMotor = LeftMotor;
        rightMotor = RightMotor;
        positioner = Positioner;
        gyro = Gyro;
        driveRatio = DriveRatio;
        floatTolerance = FloatTolerance;

        leftDriveEncoder = LeftDriveEncoder;
        rightDriveEncoder = RightDriveEncoder;

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

        currentForwardSpeed = controller.ApplyModifiers(forwardSpeed);
        currentRotationAngle = controller.ApplyModifiers(rotationAngle);

        WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(currentForwardSpeed, currentRotationAngle, true);
        myTable.getEntry("leftSpeed").setDouble(speeds.left);
        myTable.getEntry("rightSpeed").setDouble(speeds.right);
       
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

        positioner.Initialize();
    }

    public Pose3d GetPositionerOffset() {
        var adjustedPos = RobotBase.isReal() ? positioner.GetReferenceInFieldCoords() : fakeCameraPose.transformBy(GetPosition().minus(fakeCameraPose));
        var referenceAngle = RobotBase.isReal() ? positioner.GetReferenceInRobotCoords().getRotation().getZ() : fakeCameraPose.getRotation().getZ();
        return new Pose3d(adjustedPos.getTranslation(), new Rotation3d(new Rotation2d(referenceAngle)));
    }

    public void SetCurrentPose(Pose3d newPose) {
        // TODO: Implement differential drive.

        // SwerveModulePosition[] positions = new SwerveModulePosition[driveModules.size()];

        // for (int i = 0; i < driveModules.size(); i++) {
        //     SwerveMotorModule module = driveModules.get(i);
        //     positions[i] = module.getPosition();
        // }

        // poseEstimator.resetPosition(newPose.getRotation().toRotation2d(), positions, newPose.toPose2d());

        // positionInitialized = true;
        // myTable.getEntry("positionInitialized").setBoolean(positionInitialized);
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
        // TODO: Implement this.
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
    }
}
