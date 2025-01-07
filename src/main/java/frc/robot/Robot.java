// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Hashtable;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GameController.ButtonName;
import frc.robot.GameController.ControllerType;
import frc.robot.auto.AutoController;
import frc.robot.auto.SequenceMoveAndShoot;
import frc.robot.gyro.GyroBase;
import frc.robot.encoder.AnalogAbsoluteEncoder;
import frc.robot.encoder.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  final String codeBuildVersion = "2024.12.05-PreSeason";

  final PWMSparkMax m_pwm1 = new PWMSparkMax(1);
  final Talon m_pwm2 = new Talon(2);

  final Spark m_pwm3 = new Spark(3);
  final Spark m_pwm4 = new Spark(4);
  final Spark m_pwm5 = new Spark(5);
  final Spark m_pwm6 = new Spark(6);

  final PWMVictorSPX m_pwm7 = new PWMVictorSPX(7);
  final PWMVictorSPX m_pwm8 = new PWMVictorSPX(8);

  final PWMVictorSPX m_pwm9 = new PWMVictorSPX(9);
  final PWMVictorSPX m_pwm10 = new PWMVictorSPX(10);

  // JE motor is 44.4 pulses per rotation, and it reports in degrees, so there are
  // 8.108 degress per pulse.
  // not used for absolute encoders
  // final Encoder m_enc1 = new QuadEncoder(0, 1, 8.108);
  final Encoder m_enc1 = new AnalogAbsoluteEncoder(0);
  final Encoder m_enc2 = new AnalogAbsoluteEncoder(1);

  final GyroBase m_gyro = new GyroBase(9);

  GameController m_controller; // = new Controller(0, ControllerType.Xbox);

  final Timer m_timer = new Timer();

  final boolean isFieldOriented = false;

  final double m_floatTolerance = 0.08; // 0.2;
  final double m_ejectSpeed = 1.0;
  final double m_intakeSpeed = 1.0;
  final double m_liftSpeed = 1.0;
  final double m_feedSpeed = 1.0;
  // 24 teeth on driver, 42 teeth on driven = 24/42 = 0.5714
  final double m_encoderMultiplier = 1.0; // 0.5714;

  double m_divider = 0.5;
  double m_speedMod = 1.0;

  // sport gear box with 4:1 ratio on a 4" wheel yields 91.7 ft/sec which is
  // 27.95016 meters/sec
  // https://www.andymark.com/products/sport-gearbox
  // higher numbers result in faster drive speeds. To slow it down, send a higher
  // number, which will result in a lower voltage being sent to the motor for any
  // given speed.
  double m_driveSpeed = 3.721; // 5.486 / m_speedMod; // 27.95 / m_speedMod; // should be actual meters per
                               // second that is achievable by the drive motor
  // JE motor turns at 310 RPM (rotations per minute) which is 5.16 rotations per
  // second, which is 32.40 radians per second
  // https://cdn.andymark.com/media/W1siZiIsIjIwMjIvMDIvMDIvMDgvMzMvMTIvNzMzYmY3YmQtYTI0MC00ZDkyLWI5NGMtYjRlZWU1Zjc4NzY0L2FtLTQyMzNhIEpFLVBMRy00MTAgbW90b3IuUERGIl1d/am-4233a%20JE-PLG-410%20motor.PDF?sha=5387f684d4e2ce1f
  // higher numbers result in faster drive speeds. To slow it down, send a higher
  // number, which will result in a lower voltage being sent to the motor for any
  // given speed.
  // 775/redline motors run at 21,000 rpms, with a 20:1 gearbox, 1,050 rpm,
  // divided by 60 is 17.5 rotations per second, multiplied by 6.28 radians is
  // 109.9 radians per second
  // 775/redline motors run at 21,000 rpms, with a 100:1 gearbox, 210 rpm, divided
  // by 60 is 3.5 rotations per second, multiplied by 6.28 radians is 21.98
  // radians per second
  // 775/redline motors run at 21,000 rpms, with a 125:1 gearbox, 168 rpm, divided
  // by 60 is 2.8 rotations per second, multiplied by 6.28 radians is 17.584
  // radians per second
  double m_rotationSpeed = 35.168; // 17.584; // 21.98; //32.40 / m_speedMod; // should be actual radians per
                                   // second that is achievable by the rotation motor

  SingleMotorModule intake = new SingleMotorModule("intake", m_pwm5, m_intakeSpeed, false);
  DualMotorModule ejector = new DualMotorModule("ejector", m_pwm6, m_pwm7, m_ejectSpeed, true, false);
  DualMotorModule ejectorSlow = new DualMotorModule("ejectorSlow", m_pwm6, m_pwm7, m_ejectSpeed / 2, true, false);
  SingleMotorModule feeder = new SingleMotorModule("feeder", m_pwm8, m_feedSpeed, true);
  DualMotorModule lifter = new DualMotorModule("lifter", m_pwm9, m_pwm10, m_liftSpeed, true, false);
  DualMotorModule intakeUpper = new DualMotorModule("intakeUpper", m_pwm6, m_pwm7, m_ejectSpeed / 2, false, true);

  // total length of robot is 32.375", centerline is 16.1875" from edge. Drive
  // axle center is 4" from edge - 12.1875" from center which is 309.56mm or
  // 0.30956 meters
  SwerveMotorModule leftFrontMM = new SwerveMotorModule("leftFront", new Translation2d(0.30956, 0.30956), m_pwm1,
      m_pwm2, m_enc1, m_encoderMultiplier, m_floatTolerance, true, false);
  SwerveMotorModule rightRearMM = new SwerveMotorModule("rightRear", new Translation2d(-0.30956, -0.30956), m_pwm3,
      m_pwm4, m_enc2, m_encoderMultiplier, m_floatTolerance, false, false);
  SwerveDriveModule swerveDriveModule = new SwerveDriveModule("swerveDrive", m_gyro, m_driveSpeed, m_rotationSpeed,
      isFieldOriented, m_floatTolerance, leftFrontMM, rightRearMM);

  DifferentialDriveModule diffDriveModule = new DifferentialDriveModule("differentialDrive", m_pwm1, m_pwm2);

  ModuleController modules;

  Hashtable<String, AutoController> AutoModes = new Hashtable<String, AutoController>();
  AutoController currentAutoMode;

  public static final String DriveSelectionKey = "DriveSelection";
  public static final String DriveSelectionSwerve = "Swerve";
  public static final String DriveSelectionDifferential = "Differential";

  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    // SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putString("DB/String 0", codeBuildVersion);
    SmartDashboard.putNumber("DB/Slider 0", m_speedMod);

    Preferences.initString(DriveSelectionKey, DriveSelectionSwerve);
    String DriveSelection = Preferences.getString(DriveSelectionKey, DriveSelectionSwerve);

    switch (DriveSelection) {
      case DriveSelectionDifferential:
        modules = new ModuleController(diffDriveModule, m_divider);
      case DriveSelectionSwerve:
      default:
        modules = new ModuleController(swerveDriveModule, m_divider);
        break;
    }

    modules.AddModule(intake);
    modules.AddModule(ejector);
    modules.AddModule(ejectorSlow);
    modules.AddModule(feeder);
    modules.AddModule(lifter);
    modules.AddModule(intakeUpper);

    swerveDriveModule.debug = true;
    leftFrontMM.debugAngle = false;
    leftFrontMM.debugSpeed = false;

    m_enc1.setAngleOffsetDeg(25);

    JoystickIndexLoop: for (int j = 0; j < 6; j++) {
      System.out.printf("Checking for joystick on port %d\n", j);

      if (DriverStation.isJoystickConnected(j)) {
        var jtype = DriverStation.getJoystickType(j);
        System.out.printf("Joystick is connected on port %d; found type %d\n", j, jtype);
        switch (GenericHID.HIDType.of(jtype)) {
          case kXInputFlightStick, kHIDFlight:
            System.out.printf("Joystick on port %d is a FlightStick\n", j);
            m_controller = new GameController(j, ControllerType.FlightStick);
            break JoystickIndexLoop;
          default:
          case kXInputGamepad, kHIDGamepad:
            if (DriverStation.getJoystickIsXbox(j)) {
              System.out.printf("Joystick on port %d is an Xbox controller\n", j);
              m_controller = new GameController(j, ControllerType.Xbox);
              break JoystickIndexLoop;
            } else {
              System.out.printf("Joystick on port %d is not an Xbox controller, assuming PS4\n", j);
              m_controller = new GameController(j, ControllerType.PS4);
              break JoystickIndexLoop;
            }
        }
      } else {
        System.out.printf("Joystick is not connected on port %d\n", j);
      }
    }

    if (m_controller == null) {
      System.out.println("no joysticks detected!  Assuming XBox Controller on port 0");
      m_controller = new GameController(0, ControllerType.Xbox);
    }

    // three different modules operate the same component differently
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, ejector::ProcessState);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, ejectorSlow::ProcessState);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, intakeUpper::ProcessState);

    // map both of these actions to the same button
    m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, intake::ProcessState);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, feeder::ProcessState);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.BottomButton, lifter::ProcessState);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, modules::ProcessInverse);

    m_controller.RegisterValueButtonConsumer(ButtonName.LeftTrigger, modules::ProcessDivider1);

    m_controller.SetValueButtonInversion(ButtonName.RightTrigger, true);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightTrigger, modules::ProcessSpeedDilation); // ProcessSpeedLock);

    m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickY, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickY, swerveDriveModule::ProcessForwardSpeed);

    m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickX, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickX, swerveDriveModule::ProcessLateralSpeed);

    m_controller.SetValueButtonInversion(ButtonName.RightThumbstickX, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickX, swerveDriveModule::ProcessRotationAngle);

    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickY, diffDriveModule::ProcessForwardSpeed);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickX, diffDriveModule::ProcessRotationAngle);

    // m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickY,
    // modules::ProcessSpeedDilation);

    AutoController timedShoot = new AutoController("MoveAndShoot");
    timedShoot.AddSequence(new SequenceMoveAndShoot(timedShoot.GetLabel(), modules, timedShoot));
    AutoModes.put(timedShoot.GetLabel(), timedShoot);

    SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    currentAutoMode = AutoModes.get(SmartDashboard.getString("Auto Selector", AutoModes.keys().nextElement()));
    currentAutoMode.Initialize();

    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    currentAutoMode.Update();
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // empty
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());

    // get settings from dashboard
    // slider 0 is motor speed
    modules.setSpeedMod(SmartDashboard.getNumber("DB/Slider 0", 1.0));

    if (SmartDashboard.getBoolean("DB/Button 0", false))
      modules.setInverseValue(1.0);
    else
      modules.setInverseValue(-1.0);
    m_controller.ProcessButtons();
    modules.ProcessDrive();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}