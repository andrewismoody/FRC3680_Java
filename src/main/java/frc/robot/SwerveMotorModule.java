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
  long rotationLimitTime = 1000; // one second

  PIDController pidController = new PIDController(0.15, 0.0005, 0); // p=0.2

  public boolean debugAngle = false;
  public boolean debugSpeed = false;
  public boolean usePID = false;

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

    // not used for absolute encoders
    AngleEncoder.setReverseDirection(InvertRotation);
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

    var optState = SwerveModuleState.optimize(moduleState, currentAngle);  

    // slow down if we aren't aiming the right direction yet
    optState.speedMetersPerSecond *= optState.angle.minus(currentAngle).getCos();

    setAngle(optState);
    setSpeed(optState);
  }

  void setAngle(SwerveModuleState state) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://www.chiefdelphi.com/t/normal-spark-pid-p-i-and-d-values/427683/4
    var distance = currentAngle.getDegrees() + 0.0; // add zero to prevent negative zero
    var currentRad = currentAngle.getRadians() + 0.0; // add zero to prevent negative zero

    if (previousTime == 0) {
      elapsedTime = 0;
    } else {
      elapsedTime = System.currentTimeMillis() - previousTime;
    }
    previousTime = System.currentTimeMillis();

    if (Math.abs(previousDistance - distance) > floatTolerance && debugAngle) {
      //System.out.printf("%s distance: %f; previous distance: %f\n", moduleID, distance, previousDistance);
      previousDistance = distance;
    }

    if (Math.abs(previousCurrentAngle - currentRad) > floatTolerance && debugAngle) {
      System.out.printf("%s current angle: %f; previous current angle: %f\n", moduleID, currentRad, previousCurrentAngle);
      previousCurrentAngle = currentRad;
    }

    var tarAngle = state.angle;
    var tarRad = tarAngle.getRadians() + 0.0; // add 0 to prevent negative zero
    if (Math.abs(previousTargetAngle - tarRad) > floatTolerance && debugAngle) {
      System.out.printf("%s target angle radians: %f\n", moduleID, tarRad);
      previousTargetAngle = tarRad;
    }

    var delAngle = tarAngle.minus(currentAngle).getRadians() + 0.0; // add 0 to prevent negative zero
    if (Math.abs(previousDeltaAngle - delAngle) > floatTolerance && debugAngle) {
      System.out.printf("%s delta angle radians: %f\n", moduleID, delAngle);
      previousDeltaAngle = delAngle;

      // reset give up parameters
      rotationStartTime = 0;
      accumulatedMotorSpeed = 0.0;
    }

    var motorSpeed = 
      Math.abs(delAngle) > floatTolerance ?
        usePID ? pidController.calculate(delAngle, tarRad) : delAngle
      :
        0.0
    ;

    // start rotating wheel to the new optimized angle
    // get volts conversion - need to do real-world measurements to understand/identify this conversion
    // can't use this in conjunction with PID controller - not sure this is true?
    motorSpeed = motorSpeed / (usePID ? 1.0 : (driveModule.rotationSpeed * (elapsedTime / 1000)));

    if (Math.abs(previousCurrentAngle - currentRad) <= floatTolerance && Math.abs(delAngle) > floatTolerance) {
      System.out.printf("%s increasing speedincrement: %f\n", moduleID, accumulatedMotorSpeed);
      double speedIncrement = 0.1;

      accumulatedMotorSpeed = accumulatedMotorSpeed + speedIncrement;

      if (rotationStartTime == 0)
        rotationStartTime = System.currentTimeMillis();
      else if (System.currentTimeMillis() - rotationStartTime > rotationLimitTime) {
        System.out.printf("%s giving up\n", moduleID);
        motorSpeed = 0.0;
        accumulatedMotorSpeed = 0.0;
      }
    } else {
      accumulatedMotorSpeed = 0.0;
      rotationStartTime = 0;
    }

    double sign = motorSpeed > 0 ? 1 : -1;

    motorSpeed = motorSpeed + (accumulatedMotorSpeed * sign);

    if (invertRotation)
      motorSpeed *= -1;

    if (Math.abs(previousRotationSpeed - motorSpeed) > 0.001 && debugAngle) {
      System.out.printf("%s setAngle: motor speed: %f\n", moduleID, motorSpeed);
      previousRotationSpeed = motorSpeed;
    }
    
    rotatorMotor.set(motorSpeed);

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
