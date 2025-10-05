// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.s2025;

import java.util.Hashtable;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.GameController;
import frc.robot.GameController.ButtonName;
import frc.robot.GameController.ControllerType;
import frc.robot.action.*;
import frc.robot.auto.AutoController;
import frc.robot.gyro.AHRSGyro;
import frc.robot.gyro.Gyro;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.SwerveDriveModule;
import frc.robot.modules.SwerveMotorModule;
import frc.robot.positioner.LimeLightPositioner;
import frc.robot.positioner.Positioner;
import frc.robot.s2025.Sequences.SequenceControllerMoveToReef;
import frc.robot.s2025.Sequences.SequenceControllerScoreReloadScore;
import frc.robot.s2025.Sequences.SequenceRotateScoreReturn;
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

  final String codeBuildVersion = "2025.10.05-THOR";

  // RR
  final SparkMax can_drive_rr = new SparkMax(10, MotorType.kBrushless);
  final SparkMax can_steer_rr = new SparkMax(3, MotorType.kBrushless);

  // LF
  final SparkMax can_drive_lf = new SparkMax(4, MotorType.kBrushless);
  final SparkMax can_steer_lf = new SparkMax(5,MotorType.kBrushless);

  // RF
  final SparkMax can_drive_rf = new SparkMax(6, MotorType.kBrushless);
  final SparkMax can_steer_rf = new SparkMax(7, MotorType.kBrushless);

  // LR
  final SparkMax can_drive_lr = new SparkMax(8, MotorType.kBrushless);
  final SparkMax can_steer_lr = new SparkMax(9, MotorType.kBrushless);

  final SparkMax can_elev = new SparkMax(2, MotorType.kBrushless);

  final Relay pwm_slide = new Relay(0);

  // rf
  final Encoder enc_rf = new REVEncoder(can_steer_rf.getEncoder());
  // lf
  final Encoder enc_lf = new REVEncoder(can_steer_lf.getEncoder());
  // rr
  final Encoder enc_rr = new REVEncoder(can_steer_rr.getEncoder());
  // lr
  final Encoder enc_lr = new REVEncoder(can_steer_lr.getEncoder());

  final Gyro m_gyro = new AHRSGyro();

  final Encoder enc_elev = new REVEncoder(can_elev.getEncoder());

  final Positioner m_positioner = new LimeLightPositioner(false);

  GameController m_controller; // = new Controller(0, ControllerType.Xbox);

  final Timer m_timer = new Timer();

  final boolean isFieldOriented = false;

  final double m_floatTolerance = 0.08; // 0.2;
  // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
  // 100:1 gearbox on 94.6 rps = 0.946 rps shaft output
  final double m_elevatorSpeed = (5676.0 / 60.0) / 100.0;
  final double elevatorEncoderMultiplier = 1.0 / 100.0;

  final double m_liftSpeed = 0.6;
  final double m_grabSpeed = 0.6;

  double m_divider = 0.5;
  double m_speedMod = 1.0;

  // higher numbers result in faster drive speeds. To slow it down, send a higher
  // number, which will result in a lower voltage being sent to the motor for any
  // given speed.
  // 4" wheel = 0.1016m, radius = 0.0508m
  // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 * 6.28 = 35645 radians per minute; 35645 / 60 = 594 radians per second
  // 9:1 gearbox with 3:1 gear reduction (27:1 total) on 594 rps = 22 rps shaft output
  // should be actual meters per second that is achievable by the drive motor
  final double m_driveSpeed = 0.0508 * (((5676.0 * 6.28) / 60.0) / 27.0); 

  // https://cdn.andymark.com/media/W1siZiIsIjIwMjIvMDIvMDIvMDgvMzMvMTIvNzMzYmY3YmQtYTI0MC00ZDkyLWI5NGMtYjRlZWU1Zjc4NzY0L2FtLTQyMzNhIEpFLVBMRy00MTAgbW90b3IuUERGIl1d/am-4233a%20JE-PLG-410%20motor.PDF?sha=5387f684d4e2ce1f
  // higher numbers result in faster drive speeds. To slow it down, send a higher
  // number, which will result in a lower voltage being sent to the motor for any
  // given speed.
  // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 * 6.28 = 35645 radians per minute; 35645 / 60 = 594 radians per second
  // 20:1 gearbox on 594 rps = 29.7 rps shaft output
  // should be actual radians per second that is achievable by the rotation motor
  final double steerMotorSpeed = ((5676.0 * 6.28) / 60.0) / 20.0; 
  // 20:1 gearbox
  final double steeringEncoderMultiplier = 1.0 / 20.0;

  SingleMotorModule elevator = new SingleMotorModule("elevator", can_elev, m_elevatorSpeed, false, null, null, enc_elev, elevatorEncoderMultiplier, 0.5);

  SingleActuatorModule slide = new SingleActuatorModule("slide", pwm_slide, false);
  
  // total length of robot is 32.375", width is 27.5", centerline is 16.1875" from edge.  Drive axle center is 4" from edge - 12.1875" from center which is 309.56mm or 0.30956 meters
  SwerveMotorModule leftFrontMM = new SwerveMotorModule("leftFront", new Translation2d(-0.276225, -0.238125), can_drive_lf, can_steer_lf, enc_lf, steeringEncoderMultiplier, m_floatTolerance, false, true);
  SwerveMotorModule rightFrontMM = new SwerveMotorModule("rightFront", new Translation2d(0.276225, -0.238125), can_drive_rf, can_steer_rf, enc_rf, steeringEncoderMultiplier, m_floatTolerance, false, false);
  SwerveMotorModule leftRearMM = new SwerveMotorModule("leftRear", new Translation2d(-0.276225, 0.238125), can_drive_lr, can_steer_lr, enc_lr, steeringEncoderMultiplier, m_floatTolerance, false, false);
  SwerveMotorModule rightRearMM = new SwerveMotorModule("rightRear", new Translation2d(0.276225, 0.238125), can_drive_rr, can_steer_rr, enc_rr, steeringEncoderMultiplier, m_floatTolerance, false, false);

  SwerveDriveModule swerveDriveModule = new SwerveDriveModule("swerveDrive", m_gyro, m_positioner, m_driveSpeed, steerMotorSpeed, m_floatTolerance
    , leftFrontMM
    , rightFrontMM
    , leftRearMM
    , rightRearMM
  );

  // DifferentialDriveModule diffDriveModule = new DifferentialDriveModule("differentialDrive", can_steer_rr, can_drive_lf);

  ModuleController modules;

  Hashtable<String, AutoController> AutoModes = new Hashtable<String, AutoController>();
  AutoController currentAutoMode;

  public Robot() {
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

    modules = new ModuleController(swerveDriveModule, m_divider, m_controller);

    modules.AddModule(elevator);
    modules.AddModule(slide);
    
    modules.Initialize();

    // modules.SetEnableDrive(true);
    // modules.SetEnableSteer(true);
    // modules.SetEnableDriveTrain(true);

    InitializeChoosers();
    InitializeJoystick();
    InitializeButtonMappings();
    InitializeActionPoses();
    InitializeAutoModes();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    var selectedMode = AutoModes.get(SmartDashboard.getString("Auto Selector", AutoModes.keys().nextElement()));
    if (selectedMode == null)
      selectedMode = currentAutoMode; 
    selectedMode.Initialize();

    // default to field oriented for Auto
    modules.GetDriveModule().SetFieldOriented(true);

    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    currentAutoMode.Update();
    
    modules.ProcessState(true);
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // switch back to defined field oriented mode when we start up tele-op; prevents bleedover from auto
    modules.GetDriveModule().SetFieldOriented(isFieldOriented);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro", m_gyro.getAngle());

    // get settings from dashboard
    // slider 0 is motor speed
    modules.setSpeedMod(SmartDashboard.getNumber("DB/Slider 0", 1.0));

    modules.ProcessState(false);

    // TODO: figure out why these print blank values sometimes
    SmartDashboard.putString("DB/String 5", "LF: " + String.valueOf(leftFrontMM.getPosition().angle.getDegrees()));
    SmartDashboard.putString("DB/String 6", "RF: " + String.valueOf(rightFrontMM.getPosition().angle.getDegrees()));
    SmartDashboard.putString("DB/String 7", "LR: " + String.valueOf(leftRearMM.getPosition().angle.getDegrees()));
    SmartDashboard.putString("DB/String 8", "RR: " + String.valueOf(rightRearMM.getPosition().angle.getDegrees()));
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

  public void InitializeChoosers() {
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
  }

  public void InitializeJoystick() {
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
  }

  public void InitializeButtonMappings() {
    // TODO: need to move button mappings to preferences and initialize in game controller class
    
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
    m_controller.RegisterBinaryButtonConsumer(ButtonName.Select, swerveDriveModule::ReturnToZero);
    m_controller.RegisterBinaryButtonConsumer(ButtonName.Select, elevator::SetScoringPoseTrough);

    // m_controller.RegisterBinaryButtonConsumer(ButtonName.RightShoulderButton, modules::ProcessInverse);

    m_controller.RegisterValueButtonConsumer(ButtonName.LeftTrigger, modules::ProcessDivider1);

    m_controller.SetValueButtonInversion(ButtonName.RightTrigger, true);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightTrigger, modules::ProcessSpeedDilation);

    m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickY, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickY, swerveDriveModule::ProcessForwardSpeed);

    m_controller.SetValueButtonInversion(ButtonName.LeftThumbstickX, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.LeftThumbstickX, swerveDriveModule::ProcessLateralSpeed);

    m_controller.SetValueButtonInversion(ButtonName.RightThumbstickX, false);
    m_controller.RegisterValueButtonConsumer(ButtonName.RightThumbstickX, swerveDriveModule::ProcessRotationAngle);
  }

  public void InitializeActionPoses() {
    swerveDriveModule.AddActionPose(new ActionPose(Group.Score, Location.Reef, 0, Position.Any, Action.Any, new Pose3d(new Translation3d(2.65, 5.65, 0), new Rotation3d(0, 0, 5.495))));
    swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral, 0, Position.Any, Action.Any, new Pose3d(new Translation3d(2.0, 0.0, 0), new Rotation3d(0, 0, 0))));
    swerveDriveModule.AddActionPose(new ActionPose(Group.Pickup, Location.Coral, 1, Position.Any, Action.Any, new Pose3d(new Translation3d(2.0, 50.0, 0), new Rotation3d(0, 0, 315))));

    elevator.AddActionPose(new ActionPose(Group.Score, Location.Any, -1, Position.Lower, Action.Any, new Pose3d(new Translation3d(0.28, 0, 0), new Rotation3d())));
    elevator.AddActionPose(new ActionPose(Group.Score, Location.Any, -1, Position.Middle, Action.Any, new Pose3d(new Translation3d(1.14, 0, 0), new Rotation3d())));
    elevator.AddActionPose(new ActionPose(Group.Any, Location.Any, -1, Position.Trough, Action.Any, new Pose3d(new Translation3d(0.0, 0, 0), new Rotation3d())));

    slide.AddActionPose(new ActionPose(Group.Any, Location.Any, -1, Position.Any, Action.Drop, new Pose3d(new Translation3d(1, 0, 0), new Rotation3d())));
    slide.AddActionPose(new ActionPose(Group.Any, Location.Any, -1, Position.Any, Action.Pickup, new Pose3d(new Translation3d(-1, 0, 0), new Rotation3d())));
  }

  public void InitializeAutoModes() {
    // TODO: need to move this definition to preferences and initialize in automodes rather than hard coding
    AutoController rotateScoreReturn = new AutoController("RotateScoreReturn", m_controller);
    rotateScoreReturn.AddSequence(new SequenceRotateScoreReturn(rotateScoreReturn.GetLabel(), modules, rotateScoreReturn));
    AutoModes.put(rotateScoreReturn.GetLabel(), rotateScoreReturn);

    AutoController rotateWaitReturn = new AutoController("RotateWaitReturn", m_controller);
    rotateWaitReturn.AddSequence(new SequenceRotateScoreReturn(rotateWaitReturn.GetLabel(), modules, rotateWaitReturn));
    AutoModes.put(rotateWaitReturn.GetLabel(), rotateWaitReturn);

    AutoController controllerScoreReloadScore = new AutoController("ControllerScoreReloadScore", m_controller);
    controllerScoreReloadScore.AddSequence(new SequenceControllerScoreReloadScore(controllerScoreReloadScore.GetLabel(), modules, controllerScoreReloadScore));
    AutoModes.put(controllerScoreReloadScore.GetLabel(), controllerScoreReloadScore);

    AutoController controllerMoveToReef = new AutoController("controllerMoveToReef", m_controller);
    controllerMoveToReef.AddSequence(new SequenceControllerMoveToReef(controllerMoveToReef.GetLabel(), modules, controllerMoveToReef));
    AutoModes.put(controllerMoveToReef.GetLabel(), controllerMoveToReef);
    
    currentAutoMode = controllerMoveToReef;

    SmartDashboard.putStringArray("Auto List", new String[] {});
    SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));
  }
}
