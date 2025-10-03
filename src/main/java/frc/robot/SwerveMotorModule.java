package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.encoder.Encoder;

public class SwerveMotorModule {
  public Translation2d modulePosition;

  Rotation2d currentAngle = new Rotation2d();

  Translation2d currentPosition = new Translation2d();
  Translation2d previousPosition = new Translation2d();
  double currentDistance;

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

  NetworkTable myTable;

  boolean enableDecelComp = false;
  boolean enableGiveUp = false;

  public SwerveMotorModule(String ID, Translation2d Position, MotorController DriveMotor, MotorController RotationMotor, Encoder AngleEncoder, double EncoderMultiplier, double FloatTolerance, boolean InvertRotation, boolean InvertDrive) {
    moduleID = ID;

    modulePosition = Position;
    driveMotor = DriveMotor;
    rotatorMotor = RotationMotor;
    angleEncoder = AngleEncoder;
    floatTolerance = FloatTolerance;
    invertDrive = InvertDrive;

    encoderMultiplier = EncoderMultiplier;
    angleEncoder.setMultiplier(EncoderMultiplier);

    // decelFactor = driveModule.rotationSpeed / 1.5;

    // not used for absolute encoders
    AngleEncoder.setReverseDirection(InvertRotation);
  }

  public void Initialize() {
    myTable.getEntry("startupAngle").setDouble(angleEncoder.getDistance());
    angleEncoder.setZeroPosition();
    myTable.getEntry("zeroedAngle").setDouble(angleEncoder.getDistance());
    //angleEncoder.setAngleOffsetRad(angleEncoder.getRawValue());

    myTable.getEntry("encoderOffset").setDouble(angleEncoder.getAngleOffsetRad());
    myTable.getEntry("floatTolerance").setDouble(floatTolerance);
    myTable.getEntry("invertDrive").setBoolean(invertDrive);
    myTable.getEntry("encoderMultiplier").setDouble(encoderMultiplier);
    myTable.getEntry("invertRotation").setBoolean(invertRotation);
    myTable.getEntry("enableDecelComp").setBoolean(enableDecelComp);
    myTable.getEntry("enableGiveUp").setBoolean(enableGiveUp);
    myTable.getEntry("pidSetpoints").setString("kp: " + pidController.getP() + "; ki: " + pidController.getI() + "; kd: " + pidController.getD());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(currentDistance, currentAngle);
  }

  // setDriveModule happens before Initialize
  public void setDriveModule(SwerveDriveModule DriveModule) {
    driveModule = DriveModule;
    encoderSimRate = driveModule.rotationSpeed;

    var kp = driveModule.rotationSpeed / 20; // kp = 20% of motor capability
    var ki = kp / 10; // ki = 10% of kp
    var kd = ki * 3; // kd = 3 times ki
    pidController = new PIDController(kp, ki, kd);

    myTable = NetworkTableInstance.getDefault().getTable(driveModule.moduleID).getSubTable(moduleID);
  }

  public void updateModuleValues(SwerveModuleState moduleState, boolean optimize) {
    double distance = useFakeEncoder ?
      currentAngle.getDegrees()
      :
      angleEncoder.getDistance()
    ;

    currentAngle = Rotation2d.fromDegrees(distance);
    myTable.getEntry("currentAngle").setDouble(currentAngle.getRadians());

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
      
      myTable.getEntry("elapsedTime").setDouble(elapsedTime);

      if (driveModule.controller.enableSteer)
        setAngle(moduleState);
      if (driveModule.controller.enableDrive)
        setSpeed(moduleState);
    }
  }

  void setAngle(SwerveModuleState state) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://www.chiefdelphi.com/t/normal-spark-pid-p-i-and-d-values/427683/4
    var currentRad = currentAngle.getRadians() + 0.0; // add zero to prevent negative zero

    var tarAngle = state.angle;
    var tarRad = tarAngle.getRadians() + 0.0; // add 0 to prevent negative zero
    myTable.getEntry("TargetRadians").setDouble(tarRad);

    var delAngle = tarAngle.minus(currentAngle).getRadians() + 0.0; // add 0 to prevent negative zero
    myTable.getEntry("DeltaAngle").setDouble(delAngle);

    var decelDistance = primeDecelParams(currentRad);
    myTable.getEntry("decelDistance").setDouble(decelDistance);

    primeGiveUpParams(delAngle);

    // start rotating wheel to the new optimized angle
    var motorSpeed = pidController.calculate(delAngle, tarRad);
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

    myTable.getEntry("SteerMotorSpeed").setDouble(motorSpeed);
    rotatorMotor.set(motorSpeed);

    previousRotationSpeed = motorSpeed;
    previousCurrentAngle = currentRad;

    if (useFakeEncoder) {
      // fake adjust current angle to simulate encoder input
      currentAngle = Rotation2d.fromRadians((currentAngle.getRadians() + motorSpeed * encoderSimRate));
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

    myTable.getEntry("adjustmentFactor").setDouble(adjustmentFactor);

    return adjustmentFactor;
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
    
    myTable.getEntry("accumulatedMotorSpeed").setDouble(accumulatedMotorSpeed);

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
    myTable.getEntry("DriveMotorSpeed").setDouble(motorSpeed);

    if (Math.abs(previousDriveSpeed - motorSpeed) > floatTolerance && debugSpeed) {
      // System.out.printf("%s desired angle: %f; degrees %f\n", moduleID, optAngle.getRadians(), optAngle.getDegrees());
      // System.out.printf("%s current angle: %f; degrees %f\n", moduleID, currentAngle.getRadians() % 6.28, currentAngle.getDegrees() % 360);
      System.out.printf("%s rawMotorSpeed: %f\n", moduleID, rawMotorSpeed);
      System.out.printf("%s setSpeed: motor speed: %f\n", moduleID, motorSpeed);
      previousDriveSpeed = motorSpeed;
    }

    driveMotor.set(motorSpeed);

    currentDistance = rawMotorSpeed * elapsedTime;

    // set fake position
    var delta = new Translation2d(Math.cos(optAngle.getRadians()) * rawMotorSpeed * elapsedTime, Math.sin(optAngle.getRadians()) * rawMotorSpeed * elapsedTime);
    // System.out.printf("%s delta: %s\n", moduleID, delta);
    currentPosition = currentPosition.plus(delta);
    if ((Math.abs(previousPosition.minus(currentPosition).getX()) > floatTolerance || Math.abs(previousPosition.minus(currentPosition).getY()) > floatTolerance) && debugSpeed) {
      System.out.printf("%s setSpeed: currentPosition: %s\n", moduleID, currentPosition);
      previousPosition = currentPosition;
    }
  }

  public double getSpeed() {
    return previousDriveSpeed;
  }

}
