package frc.robot.modules;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry; // added
import frc.robot.encoder.Encoder;
import frc.robot.misc.Utility;
import frc.robot.misc.Utility.SwervePosition;

public class MecanumMotorModule {
  public Translation2d modulePosition;

  Translation2d currentPosition = new Translation2d();
  Translation2d previousPosition = new Translation2d();
  double currentDistance;
  double currentSpeed = 0.0;

  double previousTime;
  double elapsedTime;
  long now;

  double previousDriveSpeed;

  String moduleID;

  MotorController driveMotor;

  Encoder driveEncoder;
  public boolean useFakeEncoder = !RobotBase.isReal();
  double driveEncSimRate = 3.0;
  double driveEncSimFactor = 9.0;
  double encoderMultiplier = 1.0;

  double floatTolerance;

  boolean invertDrive = false;

  MecanumDriveModule driveModule;
  SwervePosition swervePosition;

  NetworkTable myTable;
  // Cached NT entries
  private NetworkTableEntry encoderOffsetEntry;
  private NetworkTableEntry floatToleranceEntry;
  private NetworkTableEntry invertDriveEntry;
  private NetworkTableEntry encoderMultiplierEntry;
  private NetworkTableEntry elapsedTimeEntry;
  private NetworkTableEntry driveMotorSpeedEntry;

  public MecanumMotorModule(Utility.SwervePosition MotorPosition, Translation2d Position, SwerveMotorDefinition MotorDefinition, double EncoderMultiplier, double FloatTolerance, boolean InvertDrive) {
    moduleID = MotorPosition.toString();
    swervePosition = MotorPosition;

    modulePosition = Position;
    driveMotor = MotorDefinition.driveMotor;
    driveEncoder = MotorDefinition.driveEncoder;
    floatTolerance = FloatTolerance;
    invertDrive = InvertDrive;

    encoderMultiplier = EncoderMultiplier;

  }

  public SwervePosition GetSwervePosition() {
    return swervePosition;
  }

  public void Initialize() {
    if (driveEncoder != null)
      driveEncoder.setZeroPosition();

    // instantiate entries
    encoderOffsetEntry = myTable.getEntry("encoderOffset");
    floatToleranceEntry = myTable.getEntry("floatTolerance");
    invertDriveEntry = myTable.getEntry("invertDrive");
    encoderMultiplierEntry = myTable.getEntry("encoderMultiplier");
    elapsedTimeEntry = myTable.getEntry("elapsedTime");
    driveMotorSpeedEntry = myTable.getEntry("DriveMotorSpeed");

    // initial values
    floatToleranceEntry.setDouble(floatTolerance);
    invertDriveEntry.setBoolean(invertDrive);
    encoderMultiplierEntry.setDouble(encoderMultiplier);
  }

  public void ResetDriveEncoder() {
    if (driveEncoder != null)
      driveEncoder.setZeroPosition();
  }

  public double getPosition() {
    return currentDistance;
  }

  // setDriveModule happens before Initialize
  public void setDriveModule(MecanumDriveModule DriveModule) {
    driveModule = DriveModule;
    driveEncSimRate = driveModule.driveSpeed * driveEncSimFactor;

    myTable = NetworkTableInstance.getDefault().getTable(driveModule.moduleID).getSubTable(moduleID);
  }

  public void updateModuleValues(double speed) {

    if (driveModule.controller.enableDriveTrain) {
      now = System.currentTimeMillis();

      if (previousTime == 0) {
        elapsedTime = 0;
      } else {
        elapsedTime = now - previousTime;
      }
      previousTime = now;
      
      elapsedTimeEntry.setDouble(elapsedTime);

      if (driveModule.controller.enableDrive)
        setSpeed(speed);
    }

  }

  void setSpeed(double rawMotorSpeed) {
    var motorSpeed = rawMotorSpeed;

    // convert from 'meters per second' to motor speed (normalized to 1)
    // get volts conversion - need to do real-world measurements to understand/identify this conversion
    motorSpeed = motorSpeed / driveModule.driveSpeed;
    if (invertDrive)
      motorSpeed *= -1;
    driveMotorSpeedEntry.setDouble(motorSpeed);

    previousDriveSpeed = motorSpeed;

    driveMotor.set(motorSpeed);
    currentSpeed = motorSpeed * driveModule.driveSpeed;

    if (useFakeEncoder && driveEncoder != null) {
      // fake adjust current distance to simulate encoder input
      var distanceAccumulation = rawMotorSpeed * driveEncSimRate;
      driveEncoder.appendSimValueRad(distanceAccumulation);
    }
    currentDistance = driveEncoder == null ?
      currentDistance + rawMotorSpeed * (elapsedTime / 1000) :
      driveEncoder.getRawValue() * driveModule.driveRatio; // driveRatio is wheelCircumference / gearRatio
    myTable.getEntry("rawMotorSpeed").setDouble(rawMotorSpeed);
    myTable.getEntry("currentDistance").setDouble(currentDistance);

    // set fake position
    var delta = new Translation2d(rawMotorSpeed * elapsedTime, 0);
    currentPosition = currentPosition.plus(delta);
    previousPosition = currentPosition;
  }

  public double getSpeed() {
    return previousDriveSpeed;
  }

}
