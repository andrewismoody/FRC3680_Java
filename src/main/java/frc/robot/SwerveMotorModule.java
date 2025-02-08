package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.encoder.Encoder;

public class SwerveMotorModule {
  public Translation2d modulePosition;

  Rotation2d currentAngle = new Rotation2d();

  Translation2d currentPosition = new Translation2d();
  Translation2d previousPosition = new Translation2d();
  double currentDistance;

  Long previousUpdate = System.nanoTime();

  double previousDistance;
  double previousCurrentAngle;
  double previousTargetAngle;
  double previousDeltaAngle;

  double previousTime;
  double elapsedTime;

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

  PIDController pidController = new PIDController(0.15, 0.0005, 0); // p=0.2

  public boolean debugAngle = false;
  public boolean debugSpeed = false;
  public boolean usePID = false;
  boolean gaveUp = false;

  boolean invertDrive = false;
  boolean invertRotation = false;

  SwerveDriveModule driveModule;

  public SwerveMotorModule(String ID, Translation2d Position, MotorController DriveMotor, MotorController RotationMotor, Encoder AngleEncoder, double EncoderMultipier, double FloatTolerance, boolean InvertRotation, boolean InvertDrive) {
    moduleID = ID;
    modulePosition = Position;
    driveMotor = DriveMotor;
    rotatorMotor = RotationMotor;
    angleEncoder = AngleEncoder;
    floatTolerance = FloatTolerance;
    invertDrive = InvertDrive;
    encoderMultiplier = EncoderMultipier;
    // decelFactor = driveModule.rotationSpeed / 1.5;

    // not used for absolute encoders
    AngleEncoder.setReverseDirection(InvertRotation);
  }

  public void Initialize() {
    System.out.printf("%s: offset: %f\n", moduleID, angleEncoder.getDistance());
    angleEncoder.setAngleOffsetRad(angleEncoder.getRawValue());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(currentDistance, currentAngle);
  }

  public void setDriveModule(SwerveDriveModule DriveModule) {
    driveModule = DriveModule;
    encoderSimRate = driveModule.rotationSpeed;
  }

  public void updateModuleValues(SwerveModuleState moduleState) {
    double distance = useFakeEncoder ?
      currentAngle.getDegrees()
      :
      // encoderMultiplier adjusts the encoder value to account for gearing ratios between driver and driven axles
      angleEncoder.getDistance() * encoderMultiplier
    ;

    currentAngle = Rotation2d.fromDegrees(distance);

    moduleState.optimize(currentAngle);  

    // slow down if we aren't aiming the right direction yet
    moduleState.speedMetersPerSecond *= moduleState.angle.minus(currentAngle).getCos();

    if (driveModule.controller.enableDrive) {
      setAngle(moduleState);
      setSpeed(moduleState);
    }
  }

  void setAngle(SwerveModuleState state) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://www.chiefdelphi.com/t/normal-spark-pid-p-i-and-d-values/427683/4
    var distance = currentAngle.getDegrees() + 0.0; // add zero to prevent negative zero
    var currentRad = currentAngle.getRadians() + 0.0; // add zero to prevent negative zero
    var now = System.currentTimeMillis();

    if (previousTime == 0) {
      elapsedTime = 0;
    } else {
      elapsedTime = now - previousTime;
    }
    previousTime = now;

    if (Math.abs(previousDistance - distance) > floatTolerance && debugAngle) {
      //System.out.printf("%d | %s distance: %f; previous distance: %f\n", now, moduleID, distance, previousDistance);
      previousDistance = distance;
    }

    var tarAngle = state.angle;
    var tarRad = tarAngle.getRadians() + 0.0; // add 0 to prevent negative zero
    if (Math.abs(previousTargetAngle - tarRad) > floatTolerance && debugAngle) {
      //System.out.printf("%d | %s target angle radians: %f\n", now, moduleID, tarRad);
      previousTargetAngle = tarRad;
    }

    var delAngle = tarAngle.minus(currentAngle).getRadians() + 0.0; // add 0 to prevent negative zero

    // attempt to adjust for deceleration
    // var rate = (previousCurrentAngle - currentRad) / (elapsedTime / 1000);
    // var decelDistance = rate / decelFactor;
    // distance = rate * time; remove the time factor and we only have the distance; this is how far it moved in a single time slice:
    var decelDistance = previousCurrentAngle - currentRad;

    if (Math.abs(previousDeltaAngle - delAngle) > floatTolerance) {
      if (debugAngle) {
        System.out.printf("%d | %s delta angle: %f; target angle: %f; current angle: %f; elapsed time: %f\n", now, moduleID, delAngle, tarRad, currentRad, elapsedTime / 1000);
      }
      previousDeltaAngle = delAngle;

      // reset give up parameters
      rotationStartTime = 0;
      accumulatedMotorSpeed = 0.0;
      gaveUp = false;
    }

    var motorSpeed =
      Math.abs(delAngle) > floatTolerance ?
        // usePID ? pidController.calculate(delAngle, tarRad) :
        delAngle
      :
        0.0
    ;

    // start rotating wheel to the new optimized angle
    // get volts conversion - need to do real-world measurements to understand/identify this conversion
    // can't use this in conjunction with PID controller - not sure this is true?
    motorSpeed = // usePID ? motorSpeed :
      motorSpeed * (driveModule.rotationSpeed * (elapsedTime / 1000));

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
        motorSpeed = 0.0;
        accumulatedMotorSpeed = 0.0;
      }
    } else {
      // we moved again, or the delta angle is smaller than our tolerance, reset everything
      accumulatedMotorSpeed = 0.0;
      rotationStartTime = 0;
      gaveUp = false;
    }

    double sign = motorSpeed > 0 ? 1 : -1;

    motorSpeed = motorSpeed + (accumulatedMotorSpeed * sign);

    // need to apply the inversion before this point - if we're not turning the right way, our calculations up to this point will be wrong
    // should consider inverting the target angle?
    if (invertRotation)
      motorSpeed *= -1;
    
      // shut off the motor if the target is closer than the deceleration distance
    if (Math.abs(delAngle) < Math.abs(decelDistance))
      motorSpeed = 0.0;

    if (Math.abs(previousRotationSpeed - motorSpeed) > floatTolerance) {
      if (debugAngle)
        System.out.printf("%d | %s setAngle: motor speed: %f\n", now, moduleID, motorSpeed);
      previousRotationSpeed = motorSpeed;
    }

    rotatorMotor.set(motorSpeed);

    if (Math.abs(previousCurrentAngle - currentRad) > floatTolerance && debugAngle) {
      //  System.out.printf("%d | %s current angle: %f; previous current angle: %f\n", now, moduleID, currentRad, previousCurrentAngle);
    }
    previousCurrentAngle = currentRad;

    if (useFakeEncoder) {
      // fake adjust current angle to simulate encoder input
      currentAngle = Rotation2d.fromRadians((currentAngle.getRadians() + motorSpeed * encoderSimRate));
    }
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

    if (Math.abs(previousDriveSpeed - motorSpeed) > floatTolerance && debugSpeed) {
      // System.out.printf("%s desired angle: %f; degrees %f\n", moduleID, optAngle.getRadians(), optAngle.getDegrees());
      // System.out.printf("%s current angle: %f; degrees %f\n", moduleID, currentAngle.getRadians() % 6.28, currentAngle.getDegrees() % 360);
      System.out.printf("%s rawMotorSpeed: %f\n", moduleID, rawMotorSpeed);
      System.out.printf("%s setSpeed: motor speed: %f\n", moduleID, motorSpeed);
      previousDriveSpeed = motorSpeed;
    }

    driveMotor.set(motorSpeed);

    var currentUpdate = System.nanoTime();
    var elapsedTime = (TimeUnit.NANOSECONDS.toMillis(currentUpdate - previousUpdate) / 1000.0);

    currentDistance = rawMotorSpeed * elapsedTime;

    // set fake position
    var delta = new Translation2d(Math.cos(optAngle.getRadians()) * rawMotorSpeed * elapsedTime, Math.sin(optAngle.getRadians()) * rawMotorSpeed * elapsedTime);
    // System.out.printf("%s delta: %s\n", moduleID, delta);
    currentPosition = currentPosition.plus(delta);
    if ((Math.abs(previousPosition.minus(currentPosition).getX()) > floatTolerance || Math.abs(previousPosition.minus(currentPosition).getY()) > floatTolerance) && debugSpeed) {
      System.out.printf("%s setSpeed: currentPosition: %s\n", moduleID, currentPosition);
      previousPosition = currentPosition;
    }

    previousUpdate = System.nanoTime();
  }

  public double getSpeed() {
    return previousDriveSpeed;
  }

}
