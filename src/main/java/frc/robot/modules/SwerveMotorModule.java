package frc.robot.modules;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
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

public class SwerveMotorModule {
  public Translation2d modulePosition;

  Rotation2d currentAngle = new Rotation2d();

  Translation2d currentPosition = new Translation2d();
  Translation2d previousPosition = new Translation2d();
  double currentDistance;
  SwerveModuleState currentState = new SwerveModuleState();
  double rotationOffset = 0.0;

  double previousCurrentAngle;
  double previousTargetAngle;
  double previousDeltaAngle;

  double previousTime;
  double elapsedTime;
  long now;

  double previousRotationSpeed;
  double previousDriveSpeed;

  String moduleID;

  MotorController driveMotor;
  MotorController rotatorMotor;

  Encoder angleEncoder;
  Encoder driveEncoder;
  public boolean useFakeEncoder = !RobotBase.isReal();
  double angleEncSimRate = 3.0;
  double angleEncSimFactor = 0.03;
  double driveEncSimRate = 3.0;
  double driveEncSimFactor = 9.0;
  double encoderMultiplier = 1.0;

  double floatTolerance;

  double accumulatedMotorSpeed = 0.0;
  long rotationStartTime = 0;
  long rotationLimitTime = 2 * 1000;

  int sampleCount = 0;
  int sampleMin = 100;
  double maxDistance = 0.0;

  PIDController pidController;

  public boolean debugAngle = false;
  public boolean debugSpeed = false;
  public boolean usePID = false;
  boolean gaveUp = false;

  boolean invertDrive = false;
  boolean invertRotation = false;

  SwerveDriveModule driveModule;
  SwervePosition swervePosition;

  NetworkTable myTable;
  // Cached NT entries
  private NetworkTableEntry startupAngleEntry;
  private NetworkTableEntry zeroedAngleEntry;
  private NetworkTableEntry encoderOffsetEntry;
  private NetworkTableEntry floatToleranceEntry;
  private NetworkTableEntry invertDriveEntry;
  private NetworkTableEntry encoderMultiplierEntry;
  private NetworkTableEntry invertRotationEntry;
  private NetworkTableEntry enableDecelCompEntry;
  private NetworkTableEntry enableGiveUpEntry;
  private NetworkTableEntry pidSetpointsEntry;
  private NetworkTableEntry currentAngleEntry;
  private NetworkTableEntry elapsedTimeEntry;
  private NetworkTableEntry targetRadiansEntry;
  private NetworkTableEntry pidOutputEntry;
  private NetworkTableEntry steerMotorSpeedEntry;
  private NetworkTableEntry driveMotorSpeedEntry;

  boolean enableDecelComp = false;
  boolean enableGiveUp = false;

  public SwerveMotorModule(Utility.SwervePosition SwervePosition, Translation2d Position, SwerveMotorDefinition MotorDefinition, double EncoderMultiplier, double FloatTolerance, boolean InvertRotation, boolean InvertDrive, double RotationOffset) {
    moduleID = SwervePosition.toString();
    swervePosition = SwervePosition;

    modulePosition = Position;
    driveMotor = MotorDefinition.driveMotor;
    driveEncoder = MotorDefinition.driveEncoder;
    rotatorMotor = MotorDefinition.rotatorMotor;
    angleEncoder = MotorDefinition.angleEncoder;
    floatTolerance = FloatTolerance;
    invertDrive = InvertDrive;

    rotationOffset = RotationOffset;

    encoderMultiplier = EncoderMultiplier;
    angleEncoder.setMultiplier(EncoderMultiplier);

    invertRotation = InvertRotation;
    // only invert real encoders, fake encoders don't invert because the inversion it match software to hardware behavior
    if (rotatorMotor instanceof SparkMax)
      // This sets encoder and motor inversion simultaneously, due to how Spark Max works.
      // Don't change it permanently, just set it now.
      // This won't affect other settings nor persist across power cycles, so swapping to a different configuration will be simple
      ((SparkMax)rotatorMotor).configure(new SparkMaxConfig().inverted(invertRotation), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    else {
      // not used for absolute encoders - why?
      angleEncoder.setReverseDirection(InvertRotation);
      rotatorMotor.setInverted(InvertRotation);
    }

  }

  public SwervePosition GetSwervePosition() {
    return swervePosition;
  }

  public void Initialize() {
    myTable.getEntry("startupAngle").setDouble(angleEncoder.getDistance());
    angleEncoder.setZeroPosition();
    if (driveEncoder != null)
      driveEncoder.setZeroPosition();

    // instantiate entries
    startupAngleEntry = myTable.getEntry("startupAngle");
    zeroedAngleEntry = myTable.getEntry("zeroedAngle");
    encoderOffsetEntry = myTable.getEntry("encoderOffset");
    floatToleranceEntry = myTable.getEntry("floatTolerance");
    invertDriveEntry = myTable.getEntry("invertDrive");
    encoderMultiplierEntry = myTable.getEntry("encoderMultiplier");
    invertRotationEntry = myTable.getEntry("invertRotation");
    enableDecelCompEntry = myTable.getEntry("enableDecelComp");
    enableGiveUpEntry = myTable.getEntry("enableGiveUp");
    pidSetpointsEntry = myTable.getEntry("pidSetpoints");
    currentAngleEntry = myTable.getEntry("currentAngle");
    elapsedTimeEntry = myTable.getEntry("elapsedTime");
    targetRadiansEntry = myTable.getEntry("targetRadians");
    pidOutputEntry = myTable.getEntry("pidOutput");
    steerMotorSpeedEntry = myTable.getEntry("steerMotorSpeed");
    driveMotorSpeedEntry = myTable.getEntry("DriveMotorSpeed");

    // initial values
    startupAngleEntry.setDouble(angleEncoder.getDistance());
    angleEncoder.setZeroPosition();
    zeroedAngleEntry.setDouble(angleEncoder.getDistance());
    encoderOffsetEntry.setDouble(angleEncoder.getAngleOffsetRad());
    floatToleranceEntry.setDouble(floatTolerance);
    invertDriveEntry.setBoolean(invertDrive);
    encoderMultiplierEntry.setDouble(encoderMultiplier);
    invertRotationEntry.setBoolean(invertRotation);
    enableDecelCompEntry.setBoolean(enableDecelComp);
    enableGiveUpEntry.setBoolean(enableGiveUp);
    pidSetpointsEntry.setString("kp: " + pidController.getP() + "; ki: " + pidController.getI() + "; kd: " + pidController.getD());
    myTable.getEntry("rotationOffset").setDouble(rotationOffset);
  }

  public void ResetDriveEncoder() {
    if (driveEncoder != null)
      driveEncoder.setZeroPosition();
  }

  public void ResetSteerEncoder() {
    if (angleEncoder != null)
      angleEncoder.setZeroPosition();
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(currentDistance, currentAngle);
  }

  // setDriveModule happens before Initialize
  public void setDriveModule(SwerveDriveModule DriveModule) {
    driveModule = DriveModule;
    angleEncSimRate = driveModule.rotationSpeed * angleEncSimFactor;
    driveEncSimRate = driveModule.driveSpeed * driveEncSimFactor;

    var kp = 0.333;
    var ki = 0; //kp * 0.1; // ki = 10% of kp
    var kd = 0; // ki * 3; // kd = 3 times ki
    pidController = new PIDController(kp, ki, kd);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
    // pidController.setTolerance(floatTolerance);
    
    myTable = NetworkTableInstance.getDefault().getTable(driveModule.moduleID).getSubTable(moduleID);
  }

  public SwerveModuleState updateModuleValues(SwerveModuleState moduleState, boolean optimize) {
    currentState.angle = currentAngle;
    
    double distance = angleEncoder.getDistance();
    currentAngle = Rotation2d.fromDegrees(distance).minus(Rotation2d.fromRadians(rotationOffset));

    if (optimize) {
      moduleState.optimize(currentAngle);
    }

    // slow down if we aren't aiming the right direction yet
    moduleState.speedMetersPerSecond *= moduleState.angle.minus(currentAngle).getCos();

    if (driveModule.controller.enableDriveTrain) {
      now = System.currentTimeMillis();

      if (previousTime == 0) {
        elapsedTime = 0;
      } else {
        elapsedTime = now - previousTime;
      }
      previousTime = now;
      
      elapsedTimeEntry.setDouble(elapsedTime);

      if (driveModule.controller.enableSteer)
        setAngle(moduleState);
      if (driveModule.controller.enableDrive)
        setSpeed(moduleState);
    }

    return moduleState;
  }

  void setAngle(SwerveModuleState state) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://www.chiefdelphi.com/t/normal-spark-pid-p-i-and-d-values/427683/4
    var currentRad = currentAngle.getRadians() + 0.0; // add zero to prevent negative zero
    currentAngleEntry.setDouble(Math.round(currentAngle.getRadians() * 100) / 100.0);

    // should we add offset to target or not?
    var tarAngle = state.angle;
    var tarRad = tarAngle.getRadians() + 0.0; // add 0 to prevent negative zero
    targetRadiansEntry.setDouble(tarRad);

    // start rotating wheel to the new optimized angle
    var motorSpeed = pidController.calculate(currentRad, tarRad);
    pidOutputEntry.setDouble(motorSpeed);

    steerMotorSpeedEntry.setDouble(motorSpeed);
    rotatorMotor.set(motorSpeed);

    previousRotationSpeed = motorSpeed;
    previousCurrentAngle = currentRad;

    if (useFakeEncoder) {
      // fake adjust current angle to simulate encoder input
      var angleAccumulation = motorSpeed * angleEncSimRate;
      angleEncoder.appendSimValueRad(angleAccumulation);
    }
  }

  public SwerveModuleState getCurrentState() {
    return currentState;
  }

  void setSpeed(SwerveModuleState state) {
    var rawMotorSpeed = state.speedMetersPerSecond + 0.0; // add zero to avoid negative zero
    var motorSpeed = rawMotorSpeed;
    var optAngle = state.angle;

    // convert from 'meters per second' to motor speed (normalized to 1)
    // get volts conversion - need to do real-world measurements to understand/identify this conversion
    motorSpeed = motorSpeed / driveModule.driveSpeed;
    if (invertDrive)
      motorSpeed *= -1;
    driveMotorSpeedEntry.setDouble(motorSpeed);

    previousDriveSpeed = motorSpeed;

    driveMotor.set(motorSpeed);
    currentState.speedMetersPerSecond = motorSpeed * driveModule.driveSpeed;

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
    var delta = new Translation2d(Math.cos(optAngle.getRadians()) * rawMotorSpeed * elapsedTime, Math.sin(optAngle.getRadians()) * rawMotorSpeed * elapsedTime);
    currentPosition = currentPosition.plus(delta);
    previousPosition = currentPosition;
  }

  public double getSpeed() {
    return previousDriveSpeed;
  }

}
