// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  private final PWMVictorSPX m_pwm9 = new PWMVictorSPX(9);
  private final PWMVictorSPX m_pwm10 = new PWMVictorSPX(10);

  private final Encoder m_enc1 = new Encoder(0, 1);
  private final Encoder m_enc2 = new Encoder(2, 3);

  private final double m_floatTolerance = 0.2;

  private final boolean isFieldOriented = false;
  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final Controller m_controller = new Controller(0, ControllerType.Xbox);

  private final Timer m_timer = new Timer();

  private final double m_ejectSpeed = 1.0;
  private final double m_intakeSpeed = 1.0;
  private final double m_liftSpeed = 1.0;
  private final double m_feedSpeed = 1.0;

  private double m_divider = 0.5;
  private double m_speedMod = 1.0;
  private double m_driveSpeed = 10.0 * m_speedMod;
  private double m_rotationSpeed = 3.0 * m_speedMod;
  
  private SingleMotorModule intake = new SingleMotorModule("intake", m_pwm5, m_intakeSpeed, false);
  private DualMotorModule ejector = new DualMotorModule("ejector", m_pwm6, m_pwm7, m_ejectSpeed, true, false);
  private DualMotorModule ejectorSlow = new DualMotorModule("ejectorSlow", m_pwm6, m_pwm7, m_ejectSpeed / 2, true, false);
  private SingleMotorModule feeder = new SingleMotorModule("feeder", m_pwm8, m_feedSpeed, true);
  private DualMotorModule lifter = new DualMotorModule("lifter", m_pwm9, m_pwm10, m_liftSpeed, true, false);
  private SingleMotorModule intakeUpper = new SingleMotorModule("intakeUpper", m_pwm8, m_feedSpeed, false);

  private SwerveMotorModule leftFrontMM = new SwerveMotorModule("leftFront", 0, new Translation2d(-1.0, 1.0), m_pwm1, m_pwm2, m_enc1, m_floatTolerance);
  private SwerveMotorModule rightRearMM = new SwerveMotorModule("rightRear", 1, new Translation2d(1.0, -1.0), m_pwm3, m_pwm4, m_enc2, m_floatTolerance);
  SwerveDriveModule swerveDriveModule = new SwerveDriveModule("swerveDrive", leftFrontMM, rightRearMM, m_gyro, m_driveSpeed, m_rotationSpeed, isFieldOriented);

  DifferentialDriveModule diffDriveModule = new DifferentialDriveModule("differentialDrive", m_pwm1, m_pwm2);

  ModuleController modules = new ModuleController(diffDriveModule, m_divider);

  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    // SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    modules.AddModule(intake);
    modules.AddModule(ejector);
    modules.AddModule(ejectorSlow);
    modules.AddModule(feeder);
    modules.AddModule(lifter);
    modules.AddModule(intakeUpper);

    // two different modules operate the same component differently
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, ejector::ProcessState);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, ejectorSlow::ProcessState);

    // map both of these actions to the same button
    m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, intake::ProcessState);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, feeder::ProcessState);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.BottomButton, lifter::ProcessState);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, intakeUpper::ProcessState);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, modules::ProcessInverse);

    m_controller.RegisterValueButtonConsumer(ButtonName.LeftTrigger, modules::ProcessDivider1);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightTrigger, modules::ProcessSpeedLock);

    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickY, swerveDriveModule::ProcessForwardSpeed);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickX, swerveDriveModule::ProcessLateralSpeed);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickX, swerveDriveModule::ProcessRotationAngle);

    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickY, diffDriveModule::ProcessForwardSpeed);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickX, diffDriveModule::ProcessRotationAngle);

    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickY, modules::ProcessSpeedDilation);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // // Drive for 2 seconds
    // if (m_timer.get() < 2.0) {
    //   // Drive forwards half speed, make sure to turn input squaring off
    //   m_robotDrive.arcadeDrive(0.5, 0.0, false);
    // } else {
    //   m_robotDrive.stopMotor(); // stop robot
    // }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    //empty
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {    
    m_controller.ProcessButtons();
    modules.ProcessDrive();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}