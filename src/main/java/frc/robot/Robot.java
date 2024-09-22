// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Controller.ButtonName;
import frc.robot.Controller.ControllerType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_pwm1 = new PWMSparkMax(1);
  private final PWMSparkMax m_pwm2 = new PWMSparkMax(2);

  private final Spark m_pwm3 = new Spark(3);
  private final Spark m_pwm4 = new Spark(4);
  private final Spark m_pwm5 = new Spark(5);
  private final Spark m_pwm6 = new Spark(6);

  private final PWMVictorSPX m_pwm7 = new PWMVictorSPX(7);
  private final PWMVictorSPX m_pwm8 = new PWMVictorSPX(8);

  private final Encoder m_enc1 = new Encoder(0, 1);
  private final Encoder m_enc2 = new Encoder(2, 3);

  //Change eject motor based on the needed one
  private final MotorController m_intake = m_pwm3;
  private final MotorController m_feed = m_pwm4;

  private final MotorController m_leftEject = m_pwm6;
  private final MotorController m_rightEject = m_pwm5;

  private final MotorController m_leftDrive = m_pwm1;
  private final MotorController m_rightDrive = m_pwm2;

  private final MotorController m_leftLift = m_pwm7;
  private final MotorController m_rightLift = m_pwm8;

  private final double m_floatTolerance = 0.2;
  private final double m_thumbstickDeadZone = 0.2;

  private final boolean isFieldOriented = false;
  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftDrive::set, m_rightDrive::set);

  private final Controller m_controller = new Controller(0, ControllerType.Xbox);

  private final Timer m_timer = new Timer();

  private final double m_EjectSpeed = 1.0;
  private final double m_LiftSpeed = 1.0;

  private final double m_Inverse = -1.0;
  private final double m_Divider1 = 0.5;

  private double m_InverseValue = 1.0;
  private double m_Divider1Value = 1.0;
  private double m_speedMod = 1.0;
  private boolean m_speedLock = false;
  private double m_driveSpeed = 10.0 * m_speedMod;
  private double m_rotationSpeed = 3.0 * m_speedMod;

  private double m_forwardSpeed = 0.0;
  private double m_lateralSpeed = 0.0;
  private double m_rotationAngle = 0.0;
  private double m_speedDilation = 0.0;

  private MotorModule leftFrontMM = new MotorModule("leftFront", new Translation2d(-1.0, 1.0), m_pwm1, m_pwm2, m_enc1, m_floatTolerance, m_driveSpeed, m_rotationSpeed);
  private MotorModule rightRearMM = new MotorModule("rightRear", new Translation2d(1.0, -1.0), m_pwm3, m_pwm4, m_enc2, m_floatTolerance, m_driveSpeed, m_rotationSpeed);

  private final SwerveDriveKinematics m_kin = new SwerveDriveKinematics(leftFrontMM.modulePosition, rightRearMM.modulePosition);

  private ButtonMapper m_buttonMapper = new ButtonMapper(m_controller);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive.setInverted(false);

    m_leftEject.setInverted(true);
    m_leftLift.setInverted(true);
    m_feed.setInverted(true);

    m_buttonMapper.RegisterBinaryButton(ButtonName.LeftButton, this::processEjectAmp);
    m_buttonMapper.RegisterBinaryButton(ButtonName.RightButton, this::processEjectSpeaker);
    m_buttonMapper.RegisterBinaryButton(ButtonName.TopButton, this::processIntakeFeed);
    m_buttonMapper.RegisterBinaryButton(ButtonName.BottomButton, this::processLift);
    m_buttonMapper.RegisterBinaryButton(ButtonName.LeftShoulderButton, this::processIntakeUpper);
    m_buttonMapper.RegisterBinaryButton(ButtonName.RightShoulderButton, this::processInverse);

    m_buttonMapper.RegisterValueButton(ButtonName.LeftTrigger, this::processDivider1);
    m_buttonMapper.RegisterValueButton(ButtonName.RightTrigger, this::processSpeedLock);

    m_buttonMapper.RegisterValueButton(ButtonName.RightThumbstickY, this::processForwardSpeed);
    m_buttonMapper.RegisterValueButton(ButtonName.RightThumbstickX, this::processLateralSpeed);
    m_buttonMapper.RegisterValueButton(ButtonName.LeftThumbstickX, this::processRotationAngle);
    m_buttonMapper.RegisterValueButton(ButtonName.LeftThumbstickY, this::processSpeedDilation);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    //empty
  }

  void processEjectAmp(boolean pressed) {
    if (pressed) {
      ejectAmp();
    } else {
      stopEjectAmp();
    }
  }

  void processEjectSpeaker(boolean pressed) {
    if (pressed) {
      ejectSpeaker();
    } else {
      stopEjectSpeaker();
    }
  }

  void processIntakeFeed(boolean pressed) {
    if (pressed) {
      intake();
      feed();
    } else {
      stopIntake();
      stopFeed();
    }
  }

  void processLift(boolean pressed) {
    if (pressed) {
      lift();
    } else {
      stopLift();
    }
  }

  void processIntakeUpper(boolean pressed) {
    if (pressed) {
      intakeUpper();
    } else {
      stopIntakeUpper();
    }
  }

  void processDivider1(double value) {
    if (value > 0) {
      m_Divider1Value = m_Divider1;
    } else {
      m_Divider1Value = 1.0;
    }
  }

  void processInverse(boolean pressed) {
    if (pressed) {
      m_InverseValue = m_Inverse;
    } else {
      m_InverseValue = 1.0;
    }
  }

  public void processSpeedLock(double value) {
    if (value > 0) {
      m_speedLock = true;
    } else {
      m_speedLock = false;
    }
  }

  public void processForwardSpeed(double value) {
    if (Math.abs(value) > m_thumbstickDeadZone)
      m_forwardSpeed = value;
    else
      m_forwardSpeed = 0.0;
  }

  public void processLateralSpeed(double value) {
    if (Math.abs(value) > m_thumbstickDeadZone)
      m_lateralSpeed = value;
    else
      m_lateralSpeed = 0.0;
  }

  public void processRotationAngle(double value) {
    if (Math.abs(value) > m_thumbstickDeadZone)
      m_rotationAngle = value;
    else
      m_rotationAngle = 0.0;
  }

  public void processSpeedDilation(double value) {
    if (Math.abs(value) > m_thumbstickDeadZone)
      m_speedDilation = value;
    else
      m_speedDilation = 0.0;
  }

  public void feed() {
    m_feed.set(1);
  }

  public void stopFeed() {
    m_feed.set(0);
  }

  public void lift() {
    // we only have to have one value since we're inverting the motor using software values on the controller class
    Double thisValue = applyModifiers(m_LiftSpeed);

    // we only set the values if they're different, otherwise we let them stay
    if (thisValue != m_leftLift.get())
      m_leftLift.set(thisValue);

    if (thisValue != m_rightLift.get())
      m_rightLift.set(thisValue);
  }

  public void stopLift() {
    m_leftLift.set(0);
    m_rightLift.set(0);
  }

  public void ejectAmp() {
    // we only have to have one value since we're inverting the motor using software values on the controller class
    Double thisValue = applyModifiers(m_EjectSpeed);

    // we only set the values if they're different, otherwise we let them stay
    if (thisValue != m_leftEject.get())
      m_leftEject.set(thisValue);

    if (thisValue != m_rightEject.get())
      m_rightEject.set(thisValue);
  }

  public void stopEjectAmp() {
    m_leftEject.set(0);
    m_rightEject.set(0);
  }


  public void ejectSpeaker() {
    m_leftEject.set(-1);
    m_rightEject.set(1);
  }

  public void stopEjectSpeaker() {
    m_leftEject.set(0);
    m_rightEject.set(0);
  }


  public void intake(){
    m_intake.set(1);
  }

  public void stopIntake(){
    m_intake.set(0);
  }


  public void intakeUpper() {
    // we only have to have one value since we're inverting the motor using software values on the controller class
    Double thisValue = applyModifiers(m_EjectSpeed);

    // we only set the values if they're different, otherwise we let them stay
    if (thisValue != m_leftEject.get())
      m_leftEject.set(thisValue);

    if (thisValue != m_rightEject.get())
      m_rightEject.set(thisValue);
  }

  public void stopIntakeUpper() {
    m_leftEject.set(0);
    m_rightEject.set(0);
  }

  public Double applyModifiers(Double value) {
    return applyModifiers(value, true);
  }

  public Double applyModifiers(double value, boolean affectSpeed) {
    value *= m_InverseValue;

    if (!m_speedLock && m_speedDilation < 0)
      // get the inverse of the Y value subtracted from 1 since the axis is reflected along the X axis
      // >= 0 is always full speed, anything less is a fraction of full speed starting at 1 down to 0
      m_speedMod = 1 - -m_speedDilation;

    if (affectSpeed)
      value *= m_Divider1Value * m_speedMod;
    
    return value;
  }


  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    
    m_buttonMapper.ProcessButtons();

    // m_robotDrive.arcadeDrive(m_controller.getLeftX(), m_controller.getLeftY());

    updateDriveMotors();

  }

  void updateDriveMotors() {
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.htm
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
    // https://www.chiefdelphi.com/t/set-motor-position-with-encoder/152088/3

    // set the chassis speed object according to current controller values
    double forwardSpeed = m_forwardSpeed * m_driveSpeed;
    double lateralSpeed = m_lateralSpeed * m_driveSpeed;
    double rotationSpeed = m_rotationAngle * m_rotationSpeed;

    ChassisSpeeds speeds = isFieldOriented ?
      new ChassisSpeeds(lateralSpeed, forwardSpeed, rotationSpeed)
      :
      ChassisSpeeds.fromFieldRelativeSpeeds(lateralSpeed, forwardSpeed, rotationSpeed, Rotation2d.fromDegrees(m_gyro.getAngle()))
    ;

    SwerveModuleState[] moduleStates = m_kin.toSwerveModuleStates(speeds);
    SwerveModuleState leftFront = moduleStates[0];
    SwerveModuleState rightRear = moduleStates[1];
    
    leftFrontMM.updateModuleValues(leftFront);
    rightRearMM.updateModuleValues(rightRear);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}