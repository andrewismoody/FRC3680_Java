// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Hashtable;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.GameController.ButtonName;
import frc.robot.GameController.ControllerType;
import frc.robot.action.*;
import frc.robot.auto.AutoController;
import frc.robot.gyro.AHRSGyro;
import frc.robot.gyro.Gyro;
import frc.robot.positioner.LimeLightPositioner;
import frc.robot.positioner.Positioner;
import frc.robot.encoder.AnalogAbsoluteEncoder;
import frc.robot.encoder.Encoder;
import frc.robot.encoder.REVEncoder;

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

  double elapsedTime;

  final String codeBuildVersion = "2025.02.15-PreSeason";

  // RR
  final PWMSparkMax pwm_drive_rr = new PWMSparkMax(1);
  final PWMVictorSPX pwm_steer_rr = new PWMVictorSPX(5);

  // LF
  final PWMSparkMax pwm_drive_lf = new PWMSparkMax(3);
  final PWMVictorSPX pwm_steer_lf = new PWMVictorSPX(6);

  // RF
  final PWMSparkMax pwm_drive_rf = new PWMSparkMax(2);
  final PWMVictorSPX pwm_steer_rf = new PWMVictorSPX(7);

  // LR
  final PWMSparkMax pwm_drive_lr = new PWMSparkMax(0);
  final PWMVictorSPX pwm_steer_lr = new PWMVictorSPX(4);

  final SparkMax can_elev = new SparkMax(2, MotorType.kBrushless);
  // final SparkMax can_lift = new SparkMax(4, MotorType.kBrushless);
  // final SparkMax can_grab = new SparkMax(3, MotorType.kBrushless);

  final Relay pwm_slide = new Relay(0);

  // JE motor is 44.4 pulses per rotation, and it reports in degrees, so there are
  // 8.108 degress per pulse.
  // not used for absolute encoders
  // final Encoder m_enc1 = new QuadEncoder(0, 1, 8.108);
  // rf
  final Encoder enc_rf = new AnalogAbsoluteEncoder(2);
  // lf
  final Encoder enc_lf = new AnalogAbsoluteEncoder(0);
  // rr
  final Encoder enc_rr = new AnalogAbsoluteEncoder(3);
  // lr
  final Encoder enc_lr = new AnalogAbsoluteEncoder(1);

  final Gyro m_gyro = new AHRSGyro();

  final Encoder enc_elev = new REVEncoder(can_elev.getEncoder());
  // final Encoder enc_lift = new REVEncoder(can_lift.getEncoder());
  // final Encoder enc_grabber = new REVEncoder(can_grab.getEncoder());

  final Positioner m_positioner = new LimeLightPositioner(true);

  GameController m_controller; // = new Controller(0, ControllerType.Xbox);

  final Timer m_timer = new Timer();

  final boolean isFieldOriented = false;

  final double m_floatTolerance = 0.08; // 0.2;
  final double m_elevatorSpeed = 0.6;
  final double m_liftSpeed = 0.6;
  final double m_grabSpeed = 0.6;
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

  SingleMotorModule elevator = new SingleMotorModule("elevator", can_elev, m_elevatorSpeed, false, null, null, enc_elev);
  // SingleMotorModule lifter = new SingleMotorModule("lifter", can_lift, m_liftSpeed, true, null, null, enc_lift);
  // SingleMotorModule grabber = new SingleMotorModule("grabber", can_grab, m_grabSpeed, false, null, null, enc_grabber);

  SingleActuatorModule slide = new SingleActuatorModule("slide", pwm_slide, false);
  
  // total length of robot is 32.375", width is 27.5", centerline is 16.1875" from edge.  Drive axle center is 4" from edge - 12.1875" from center which is 309.56mm or 0.30956 meters
  SwerveMotorModule leftFrontMM = new SwerveMotorModule("leftFront", new Translation2d(-0.30956, -0.24765), pwm_drive_lf, pwm_steer_lf, enc_lf, m_encoderMultiplier, m_floatTolerance, true, false);
  SwerveMotorModule rightFrontMM = new SwerveMotorModule("rightFront", new Translation2d(0.30956, -0.24765), pwm_drive_rf, pwm_steer_rf, enc_rf, m_encoderMultiplier, m_floatTolerance, true, false);
  SwerveMotorModule leftRearMM = new SwerveMotorModule("leftRear", new Translation2d(-0.30956, 0.24765), pwm_drive_lr, pwm_steer_lr, enc_lr, m_encoderMultiplier, m_floatTolerance, true, false);
  SwerveMotorModule rightRearMM = new SwerveMotorModule("rightRear", new Translation2d(0.30956, 0.24765), pwm_drive_rr, pwm_steer_rr, enc_rr, m_encoderMultiplier, m_floatTolerance, true, false);

  SwerveDriveModule swerveDriveModule = new SwerveDriveModule("swerveDrive", m_gyro, m_positioner, m_driveSpeed, m_rotationSpeed, isFieldOriented, m_floatTolerance
    , leftFrontMM
    , rightFrontMM
    , leftRearMM
    , rightRearMM
  );

  DifferentialDriveModule diffDriveModule = new DifferentialDriveModule("differentialDrive", pwm_steer_rr, pwm_drive_lf);

  ModuleController modules;

  Hashtable<String, AutoController> AutoModes = new Hashtable<String, AutoController>();
  AutoController currentAutoMode;

  public static final String DriveSelectionKey = "DriveSelection";
  public static final String DriveSelectionSwerve = "Swerve";
  public static final String DriveSelectionDifferential = "Differential";



  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    // SendableRegistry.addChild(m_robotDrive, m_rightDrive);
    
        CameraServer.startAutomaticCapture();
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

    SendableChooser<Group> GroupChooser = new SendableChooser<>();
    for (Group item : Group.class.getEnumConstants()) {
      GroupChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(GroupChooser);

    SendableChooser<Location> LocationChooser = new SendableChooser<>();
    for (Location item : Location.class.getEnumConstants()) {
      LocationChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(LocationChooser);

    SendableChooser<Integer> IndexChooser = new SendableChooser<>();
    for (int i = 0; i < 8; i++) {
      IndexChooser.addOption(String.valueOf(i), i);
    }
    SmartDashboard.putData(IndexChooser);

    SendableChooser<Position> PositionChooser = new SendableChooser<>();
    for (Position item : Position.class.getEnumConstants()) {
      PositionChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(PositionChooser);

    SendableChooser<Action> ActionChooser = new SendableChooser<>();
    for (Action item : Action.class.getEnumConstants()) {
      ActionChooser.addOption(item.toString(), item);
    }
    SmartDashboard.putData(ActionChooser);

    swerveDriveModule.debug = false;
    leftRearMM.debugAngle = true;
    leftFrontMM.debugSpeed = false;

    elevator.debug = true;

    

    // // lf
    // m_enc2.setAngleOffsetDeg(149);
    // // rf
    // m_enc1.setAngleOffsetDeg(114);
    // // lr
    // m_enc4.setAngleOffsetDeg(134);
    // // rr
    // m_enc3.setAngleOffsetDeg(103);

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

    switch (DriveSelection) {
      case DriveSelectionDifferential:
        modules = new ModuleController(diffDriveModule, m_divider, m_controller);
      case DriveSelectionSwerve:
      default:
        modules = new ModuleController(swerveDriveModule, m_divider, m_controller);
        break;
    }

    elevator.AddActionPose(new ActionPose(Group.Score, Location.Any, -1, Position.Lower, Action.Any, new Pose3d(new Translation3d(50.0, 0, 0), new Rotation3d())));
    elevator.AddActionPose(new ActionPose(Group.Score, Location.Any, -1, Position.Middle, Action.Any, new Pose3d(new Translation3d(130.0, 0, 0), new Rotation3d())));
    elevator.AddActionPose(new ActionPose(Group.Score, Location.Any, -1, Position.Trough, Action.Any, new Pose3d(new Translation3d(0.0, 0, 0), new Rotation3d())));
    modules.AddModule(elevator);
  //  modules.AddModule(lifter);
  //   modules.AddModule(grabber);
    modules.AddModule(slide);

    modules.enableDrive = true;
    
    modules.Initialize();


    // TODO: need to move button mappings to preferences and initialize in game controller class

    // three different modules operate the same component differently
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, ejector::ApplyValue);
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, ejectorSlow::ApplyValue);
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, intakeUpper::ApplyValue);

    // map both of these actions to the same button
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, intake::ApplyValue);
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, feeder::ApplyValue);
    
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftShoulderButton, slide::ApplyValue);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, slide::ApplyInverse);

    m_controller.RegisterBinaryButtonConsumer(ButtonName.BottomButton, elevator::SetNoPose);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.TopButton, elevator::SetScoringPoseMiddle);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, elevator::SetScoringPoseLower);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, elevator::SetScoringPoseTrough);



    m_controller.RegisterBinaryButtonConsumer(ButtonName.POVDown, elevator::ApplyInverse);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.POVUp, elevator::ApplyValue);

    // m_controller.RegisterBinaryButtonConsumer(ButtonName.POVLeft, grabber::ApplyInverse);
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.POVRight, grabber::ApplyValue);

    // m_controller.RegisterBinaryButtonConsumer(ButtonName.LeftButton, swerveDriveModule::LockPosition);
    // m_controller.RegisterBinaryButtonConsumer(ButtonName.RightButton, swerveDriveModule::ReturnToZero);

    // m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, modules::ProcessInverse);

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

    // TODO: need to move this definition to preferences and initialize in automodes rather than hard coding
    AutoController timedShoot = new AutoController("MoveAndShoot");
    //timedShoot.AddSequence(new SequenceMoveAndShoot(timedShoot.GetLabel(), modules, timedShoot));
    AutoModes.put(timedShoot.GetLabel(), timedShoot);
    //currentAutoMode = timedShoot;


    SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    //currentAutoMode = AutoModes.get(SmartDashboard.getString("Auto Selector", AutoModes.keys().nextElement()));
    //currentAutoMode.Initialize();

    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   // currentAutoMode.Update();
   swerveDriveModule.ProcessForwardSpeed(0.5);

    modules.ProcessState(true);

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());

    // get settings from dashboard
    // slider 0 is motor speed
    modules.setSpeedMod(SmartDashboard.getNumber("DB/Slider 0", 1.0));

    // if (SmartDashboard.getBoolean("DB/Button 0", false))
    //   modules.setInverseValue(1.0);
    // else
    //   modules.setInverseValue(-1.0);

    modules.ProcessState(false);

    // TODO: figure out why these print blank values sometimes
    SmartDashboard.putString("DB/String 5", "LF: " + String.valueOf(leftFrontMM.currentAngle.getDegrees()));
    SmartDashboard.putString("DB/String 6", "RF: " + String.valueOf(rightFrontMM.currentAngle.getDegrees()));
    SmartDashboard.putString("DB/String 7", "LR: " + String.valueOf(leftRearMM.currentAngle.getDegrees()));
    SmartDashboard.putString("DB/String 8", "RR: " + String.valueOf(rightRearMM.currentAngle.getDegrees()));
    SmartDashboard.putString("DB/String 9", "gyro: " + String.valueOf(m_gyro.getAngle()));
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
