package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MotorModule {
  public Translation2d modulePosition;

  double currentAngle;
  Translation2d currentPosition = new Translation2d();

  double previousCurrentAngle;
  double previousTargetAngle;
  double previousDeltaAngle;
  Translation2d previousCurrentPosition = new Translation2d();

  double rotationSpeed;
  double driveSpeed;

  double previousRotationSpeed;
  double previousDriveSpeed;

  String moduleID;

  MotorController driveMotor;
  MotorController rotatorMotor;

  Encoder angleEncoder;
  boolean useFakeEncoder = true;
  double encoderSimRate = 3.0;

  double floatTolerance;

  double previousEncoderAngle;

  PIDController pidController = new PIDController(0.2, 0, 0);

  boolean debugAngle = false;
  boolean debugSpeed = true;

  public MotorModule(String id, Translation2d position, MotorController dMot, MotorController rMot, Encoder enc, double aTol, double drvSpeed, double rotSpeed) {
    moduleID = id;
    modulePosition = position;
    driveMotor = dMot;
    rotatorMotor = rMot;
    angleEncoder = enc;
    floatTolerance = aTol;
    driveSpeed = drvSpeed;
    rotationSpeed = rotSpeed;
  }

  public void updateModuleValues(SwerveModuleState state) {
    setAngle(state);
    setSpeed(state);
  }

  void setAngle(SwerveModuleState state) {
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html
    // https://www.chiefdelphi.com/t/normal-spark-pid-p-i-and-d-values/427683/4
    double distance = useFakeEncoder ?
      currentAngle
      :
      angleEncoder.getDistance()
    ;

    var newAngle = Rotation2d.fromRadians(distance);
    var stateOpt = SwerveModuleState.optimize(state, newAngle);
    // slow down if we aren't aiming the right direction yet
    stateOpt.speedMetersPerSecond *= stateOpt.angle.minus(newAngle).getCos();

    if (Math.abs(previousCurrentAngle - currentAngle) > floatTolerance && debugAngle) {
      System.out.printf("%s current angle: %f; previous current angle: %f\n", moduleID, currentAngle, previousCurrentAngle);
      previousCurrentAngle = currentAngle;
    }
    var tarAngle = stateOpt.angle.getRadians();
    if (Math.abs(previousTargetAngle - tarAngle) > floatTolerance && debugAngle) {
      System.out.printf("%s target angle radians: %f\n", moduleID, tarAngle);
      previousTargetAngle = tarAngle;
    }
    var delAngle = Math.abs(stateOpt.angle.minus(newAngle).getRadians());
    if (Math.abs(previousDeltaAngle - delAngle) > floatTolerance && debugAngle) {
      System.out.printf("%s delta angle radians: %f\n", moduleID, delAngle);
      previousDeltaAngle = delAngle;
    }

    var motorSpeed = 
      Math.abs(stateOpt.angle.minus(newAngle).getRadians()) > floatTolerance ?
      (stateOpt.angle.getRadians() > 0 ? 1.0 : -1.0) * pidController.calculate(stateOpt.angle.minus(newAngle).getRadians())
      :
      0.0
    ;
    if (Math.abs(previousRotationSpeed - motorSpeed) > floatTolerance && debugAngle) {
      System.out.printf("%s setAngle: motor speed: %f\n", moduleID, motorSpeed);
      previousRotationSpeed = motorSpeed;
    }

    // start rotating wheel to the new optimized angle
    motorSpeed = motorSpeed / rotationSpeed;
    rotatorMotor.set(motorSpeed);

    // fake adjust current angle to simulate encoder input
    currentAngle = currentAngle + motorSpeed * encoderSimRate;
  }

  void setSpeed(SwerveModuleState state) {
    var motorSpeed = state.speedMetersPerSecond;
    if (Math.abs(previousDriveSpeed - motorSpeed) > floatTolerance && debugSpeed) {
      System.out.printf("%s setSpeed: motor speed: %f\n", moduleID, motorSpeed);
      previousDriveSpeed = motorSpeed;
    }

    // convert from 'meters per second' to motor speed (normalized to 1)
    motorSpeed = motorSpeed / driveSpeed;
    driveMotor.set(motorSpeed);

    // set fake position
    var delta = new Translation2d(Math.cos(state.angle.getRadians()) * state.speedMetersPerSecond * 0.2, Math.sin(state.angle.getRadians()) * state.speedMetersPerSecond * 0.2);
    // System.out.printf("%s delta: %s\n", moduleID, delta);
    currentPosition = currentPosition.plus(delta);
    if ((Math.abs(previousCurrentPosition.minus(currentPosition).getX()) > floatTolerance || Math.abs(previousCurrentPosition.minus(currentPosition).getY()) > floatTolerance) && debugSpeed) {
      System.out.printf("%s setSpeed: currentPosition: %s\n", moduleID, currentPosition);
      previousCurrentPosition = currentPosition;
    }
  }

}
