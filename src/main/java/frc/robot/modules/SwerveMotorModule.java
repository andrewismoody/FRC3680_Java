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
  public boolean useFakeEncoder = !RobotBase.isReal();
  double encoderSimRate = 3.0;
  double encoderSimFactor = 0.03;
  double encoderMultiplier = 1.0;

  double floatTolerance;

  double previousEncoderAngle;
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
  private NetworkTableEntry deltaAngleEntry;
  private NetworkTableEntry decelDistanceEntry;
  private NetworkTableEntry pidOutputEntry;
  private NetworkTableEntry steerMotorSpeedEntry;
  private NetworkTableEntry adjustmentFactorEntry;
  private NetworkTableEntry accumulatedMotorSpeedEntry;
  private NetworkTableEntry driveMotorSpeedEntry;

  boolean enableDecelComp = false;
  boolean enableGiveUp = false;

  public SwerveMotorModule(Utility.SwervePosition SwervePosition, Translation2d Position, MotorController DriveMotor, MotorController RotationMotor, Encoder AngleEncoder, double EncoderMultiplier, double FloatTolerance, boolean InvertRotation, boolean InvertDrive, double RotationOffset) {
    moduleID = SwervePosition.toString();
    swervePosition = SwervePosition;

    modulePosition = Position;
    driveMotor = DriveMotor;
    rotatorMotor = RotationMotor;
    angleEncoder = AngleEncoder;
    floatTolerance = FloatTolerance;
    invertDrive = InvertDrive;

    rotationOffset = RotationOffset;

    encoderMultiplier = EncoderMultiplier;
    angleEncoder.setMultiplier(EncoderMultiplier);

    // decelFactor = driveModule.rotationSpeed / 1.5;

    invertRotation = InvertRotation;
    if (RotationMotor instanceof SparkMax)
      ((SparkMax)RotationMotor).configure(new SparkMaxConfig().inverted(invertRotation), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    else
      // not used for absolute encoders - why?
      AngleEncoder.setReverseDirection(InvertRotation);

  }

  public SwervePosition GetSwervePosition() {
    return swervePosition;
  }

  public void Initialize() {
    myTable.getEntry("startupAngle").setDouble(angleEncoder.getDistance());
    angleEncoder.setZeroPosition();

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
    deltaAngleEntry = myTable.getEntry("deltaAngle");
    decelDistanceEntry = myTable.getEntry("decelDistance");
    pidOutputEntry = myTable.getEntry("pidOutput");
    steerMotorSpeedEntry = myTable.getEntry("steerMotorSpeed");
    adjustmentFactorEntry = myTable.getEntry("adjustmentFactor");
    accumulatedMotorSpeedEntry = myTable.getEntry("accumulatedMotorSpeed");
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

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(currentDistance, currentAngle);
  }

  // setDriveModule happens before Initialize
  public void setDriveModule(SwerveDriveModule DriveModule) {
    driveModule = DriveModule;
    encoderSimRate = driveModule.rotationSpeed * encoderSimFactor;

    var kp = 0.333;
    var ki = 0; //kp * 0.1; // ki = 10% of kp
    var kd = 0; // ki * 3; // kd = 3 times ki
    pidController = new PIDController(kp, ki, kd);
    // pidController.enableContinuousInput(-Math.PI, Math.PI);
    // pidController.setTolerance(floatTolerance);
    
    myTable = NetworkTableInstance.getDefault().getTable(driveModule.moduleID).getSubTable(moduleID);
  }

  public SwerveModuleState updateModuleValues(SwerveModuleState moduleState, boolean optimize) {
    currentState.angle = currentAngle;
    
    double distance = useFakeEncoder ?
      currentAngle.getDegrees()
      :
      angleEncoder.getDistance()
    ;

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

    var delAngle = tarAngle.minus(currentAngle).getRadians() + 0.0; // add 0 to prevent negative zero
    deltaAngleEntry.setDouble(delAngle);

    var decelDistance = primeDecelParams(currentRad);
    decelDistanceEntry.setDouble(decelDistance);

    primeGiveUpParams(delAngle);

    // start rotating wheel to the new optimized angle
    var motorSpeed = pidController.calculate(currentRad, tarRad);
    pidOutputEntry.setDouble(motorSpeed);

    double sign = motorSpeed > 0 ? 1 : -1;

    if (enableDecelComp)
      motorSpeed *= getAdjustmentFactor(delAngle);

    if (enableGiveUp) {
      if (gaveUp)
        motorSpeed = 0.0;
      else
        motorSpeed += (getAccumulatedMotorSpeed(currentRad, delAngle) * sign);
    }

    // need to apply the inversion before this point - if we're not turning the right way, our calculations up to this point will be wrong
    // should consider inverting the target angle?
    if (invertRotation)
      motorSpeed *= -1;
    
    if (enableDecelComp) {
      // shut off the motor if the target is closer than the deceleration distance
      if (Math.abs(delAngle) < Math.abs(decelDistance))
        motorSpeed = 0.0;
    }

    steerMotorSpeedEntry.setDouble(motorSpeed);
    rotatorMotor.set(motorSpeed);

    previousRotationSpeed = motorSpeed;
    previousCurrentAngle = currentRad;

    if (useFakeEncoder) {
      // fake adjust current angle to simulate encoder input
      currentAngle = Rotation2d.fromRadians((currentAngle.plus(Rotation2d.fromRadians(rotationOffset)).getRadians() + motorSpeed * encoderSimRate));
    }
  }

  double primeDecelParams(double currentRad) {
    // attempt to adjust for deceleration
    // var rate = (previousCurrentAngle - currentRad) / (elapsedTime / 1000);
    // var decelDistance = rate / decelFactor;
    // distance = rate * time; remove the time factor and we only have the distance; this is how far it moved in a single time slice:
    var decelDistance = previousCurrentAngle - currentRad;
    if (decelDistance > maxDistance) {
      maxDistance = decelDistance;
      sampleCount++;
    }
    return decelDistance;
  }

  void primeGiveUpParams(double delAngle) {
    if (Math.abs(previousDeltaAngle - delAngle) > floatTolerance) {

      // reset give up parameters
      rotationStartTime = 0;
      accumulatedMotorSpeed = 0.0;
      gaveUp = false;
    }

    previousDeltaAngle = delAngle;
  }

  double getAdjustmentFactor(double delAngle) {
    var adjustmentFactor = 1.0;

    // only apply the adjustment factor if we've had enough samples to determine a max distance
    if (sampleCount > sampleMin && delAngle < maxDistance) {
      adjustmentFactor = (delAngle / maxDistance);
    }

    adjustmentFactorEntry.setDouble(adjustmentFactor);

    return adjustmentFactor;
  }

  public SwerveModuleState getCurrentState() {
    return currentState;
  }

  double getAccumulatedMotorSpeed(double currentRad, double delAngle) {
    // if we haven't moved, and our delta angle is larger than float tolerance, boost the motor voltage
    if (Math.abs(previousCurrentAngle - currentRad) <= floatTolerance && Math.abs(delAngle) > floatTolerance) {
      double speedIncrement = 0.01;

      accumulatedMotorSpeed = accumulatedMotorSpeed + speedIncrement;
      // if (debugAngle)
      //   System.out.printf("%d | %s increasing speedincrement: %f\n", now, moduleID, accumulatedMotorSpeed);

      if (rotationStartTime == 0)
        rotationStartTime = System.currentTimeMillis();
      else if (System.currentTimeMillis() - rotationStartTime > rotationLimitTime) {
        // don't keep trying if it doesn't move - don't want to burn up the motor
        if (!gaveUp && debugAngle)
          System.out.printf("%d | %s giving up\n", now, moduleID);
        gaveUp = true;
        accumulatedMotorSpeed = 0.0;
      }
    } else {
      // we moved again, or the delta angle is smaller than our tolerance, reset everything
      accumulatedMotorSpeed = 0.0;
      rotationStartTime = 0;
      gaveUp = false;
    }
    
    accumulatedMotorSpeedEntry.setDouble(accumulatedMotorSpeed);

    return accumulatedMotorSpeed;
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

    // TODO 1: look at using encoders to get shaft rotation converted to actual wheel motion
    currentDistance += rawMotorSpeed * (elapsedTime / 1000);
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
