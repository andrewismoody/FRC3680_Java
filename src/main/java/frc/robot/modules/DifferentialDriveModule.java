package frc.robot.modules;

import java.util.ArrayList;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.action.Group;
import frc.robot.gyro.Gyro;
import frc.robot.positioner.Positioner;
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

    NetworkTable myTable;

    public DifferentialDriveModule(String ModuleID, Gyro Gyro, Positioner Positioner, MotorController LeftMotor, MotorController RightMotor) {
        moduleID = ModuleID;
        leftMotor = LeftMotor;
        rightMotor = RightMotor;
        positioner = Positioner;
        gyro = Gyro;

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
    }

    public void ResetGyro() {
        gyro.reset();
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
        currentForwardSpeed = controller.ApplyModifiers(forwardSpeed);
        if (debugSpeed && forwardSpeed != previousForwardSpeed) {
            System.out.printf("%s forwardSpeed: %f\n", moduleID, forwardSpeed);
            previousForwardSpeed = forwardSpeed;
        }

        currentRotationAngle = controller.ApplyModifiers(rotationAngle);
        if (debugAngle && rotationAngle != previousRotationAngle) {
            System.out.printf("%s rotationAngle: %f\n", moduleID, rotationAngle);
            previousRotationAngle = rotationAngle;
        }

        driveController.arcadeDrive(currentForwardSpeed, currentRotationAngle);

        // update dashboard
        SmartDashboard.putNumberArray("RobotDrive Motors", new double[]{leftMotor.get(), rightMotor.get()});
    }

    public void Initialize() {

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
