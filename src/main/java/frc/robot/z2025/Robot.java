// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.z2025;

import java.util.Hashtable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.auto.AutoController;
import frc.robot.gyro.AHRSGyro;
import frc.robot.gyro.Gyro;
import frc.robot.misc.GameController;
import frc.robot.misc.Utility;
import frc.robot.misc.Utility.SwervePosition;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleActuatorModule;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.SwerveDriveModule;
import frc.robot.modules.SwerveMotorModule;
import frc.robot.positioner.LimeLightPositioner;
import frc.robot.positioner.Positioner;
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
  boolean initialized = false;

  // LF
  final SparkMax can_drive_lf = new SparkMax(4, MotorType.kBrushless);
  final SparkMax can_steer_lf = new SparkMax(5,MotorType.kBrushless);

  // RF
  final SparkMax can_drive_rf = new SparkMax(6, MotorType.kBrushless);
  final SparkMax can_steer_rf = new SparkMax(7, MotorType.kBrushless);

  // LR
  final SparkMax can_drive_lr = new SparkMax(8, MotorType.kBrushless);
  final SparkMax can_steer_lr = new SparkMax(9, MotorType.kBrushless);

  // RR
  final SparkMax can_drive_rr = new SparkMax(10, MotorType.kBrushless);
  final SparkMax can_steer_rr = new SparkMax(3, MotorType.kBrushless);

  final SparkMax can_elev = new SparkMax(2, MotorType.kBrushless);

  final Relay pwm_slide = new Relay(0);

  // lf
  final Encoder enc_steer_lf = new REVEncoder(can_steer_lf.getEncoder());
  // rf
  final Encoder enc_steer_rf = new REVEncoder(can_steer_rf.getEncoder());
  // lr
  final Encoder enc_steer_lr = new REVEncoder(can_steer_lr.getEncoder());
  // rr
  final Encoder enc_steer_rr = new REVEncoder(can_steer_rr.getEncoder());

  // lf
  final Encoder enc_drive_lf = new REVEncoder(can_drive_lf.getEncoder());
  // rf
  final Encoder enc_drive_rf = new REVEncoder(can_drive_rf.getEncoder());
  // lr
  final Encoder enc_drive_lr = new REVEncoder(can_drive_lr.getEncoder());
  // rr
  final Encoder enc_drive_rr = new REVEncoder(can_drive_rr.getEncoder());

  final Gyro m_gyro = new AHRSGyro();

  final Encoder enc_elev = new REVEncoder(can_elev.getEncoder());

  final Positioner m_positioner = new LimeLightPositioner(true);

  GameController m_controller = null;

  final Timer m_timer = new Timer();
  final Timer gc_timer = new Timer();

  final boolean isFieldOriented = true;

  final double m_floatTolerance = 0.04; // 0.2;
  // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
  // 100:1 gearbox on 94.6 rps = 0.946 rps shaft output
  final double m_elevatorSpeed = (5676.0 / 60.0) / 100.0;
  final double elevatorEncoderMultiplier = 1.0 / 100.0;

  final double m_liftSpeed = 0.6;
  final double m_grabSpeed = 0.6;

  double m_divider = 0.5;
  double m_speedMod = 1.0;

  final double driveGearRatio = 27.0;
  // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 / 60 = 94.6 rotations per second
  final double driveMotorRPM = 5676.0;
   // 4" wheel = 0.1016m diameter
  final double wheelDiameter = 0.1016;
  // 0.319m wheel circumference
  final double wheelCircumference = Math.PI * wheelDiameter;
  // 9:1 gearbox with 3:1 gear reduction (27:1 total) on 0.319m circumference = 0.0118 meters per motor rotation
  final double m_driveRatio = wheelCircumference / driveGearRatio; 
  // m_driveSpeed should be actual meters per second that is achievable by the drive motor
  // higher numbers result in faster drive speeds. To slow it down, send a higher
  // number, which will result in a lower voltage being sent to the motor for any
  // given speed.
  // 0.0118 meters per rotation * 94.6 rotations per second = 1.116 meters per second
  final double m_driveSpeed = m_driveRatio * (driveMotorRPM / 60.0); 

  // https://cdn.andymark.com/media/W1siZiIsIjIwMjIvMDIvMDIvMDgvMzMvMTIvNzMzYmY3YmQtYTI0MC00ZDkyLWI5NGMtYjRlZWU1Zjc4NzY0L2FtLTQyMzNhIEpFLVBMRy00MTAgbW90b3IuUERGIl1d/am-4233a%20JE-PLG-410%20motor.PDF?sha=5387f684d4e2ce1f
  // higher numbers result in faster drive speeds. To slow it down, send a higher
  // number, which will result in a lower voltage being sent to the motor for any
  // given speed.
  // Rev NEO empirical motor speed = 5676 rotations per minute; 5676 * 6.28 = 35645 radians per minute; 35645 / 60 = 594 radians per second
  // 20:1 gearbox on 594 rps = 29.7 rps shaft output
  // should be actual radians per second that is achievable by the rotation motor
  final double steerGearRatio = 20.0;
  final double steerMotorRPM = 5676.0;
  final double steerMotorSpeed = ((steerMotorRPM * (Math.PI * 2)) / 60.0) / steerGearRatio; 
  // 20:1 gearbox
  final double steeringEncoderMultiplier = 1.0 / steerGearRatio;

  SingleMotorModule elevator = new SingleMotorModule("elevator", can_elev, m_elevatorSpeed, false, null, null, enc_elev, elevatorEncoderMultiplier, 0.5);

  SingleActuatorModule slide = new SingleActuatorModule("slide", pwm_slide, false);
  
  // Field Dimensions Y (width) = 8.05, X (length) = 17.55; 
  Translation2d fieldSize = new Translation2d(17.55, 8.05);
  // starting line is at X = 7.56m; barge is 3.72m wide
  Translation2d startArea = new Translation2d(fieldSize.getX() / 2 - 7.56, 3.72);

  // leftFront  software position should be leftrear   hardware position
  // rightFront software position should be leftFront  hardware position
  // leftRear   software position should be rightRear  hardware position
  // rightRear  software position should be rightFront hardware position

  // flipped x and y so that 'narrow' edge is front
  Translation2d frameSize = new Translation2d(0.6985, 0.822325); // meters - 32.375" x 27.5" - distance from center of robot to center of wheel

  // total length of robot is 32.375", width is 27.5", centerline is 16.1875" from edge.  Drive axle center is 4" from edge - 12.1875" from center which is 309.56mm or 0.30956 meters
  SwerveDriveModule swerveDriveModule = new SwerveDriveModule("swerveDrive", m_gyro, m_positioner, m_driveSpeed, m_driveRatio, steerMotorSpeed, m_floatTolerance
    , new SwerveMotorModule(SwervePosition.LeftFront, new Translation2d(frameSize.getX() / 2, frameSize.getY() / 2), can_drive_lr, enc_drive_lr, can_steer_lr, enc_steer_lr, steeringEncoderMultiplier, m_floatTolerance, false, false, 0.0)
    , new SwerveMotorModule(SwervePosition.RightFront, new Translation2d(frameSize.getX() / 2, -frameSize.getY() / 2), can_drive_lf, enc_drive_lf, can_steer_lf, enc_steer_lf, steeringEncoderMultiplier, m_floatTolerance, false, false, 0.0)
    , new SwerveMotorModule(SwervePosition.LeftRear, new Translation2d(-frameSize.getX() / 2, frameSize.getY() / 2), can_drive_rr, enc_drive_rr, can_steer_rr, enc_steer_rr, steeringEncoderMultiplier, m_floatTolerance, false, false, 0.0)
    , new SwerveMotorModule(SwervePosition.RightRear, new Translation2d(-frameSize.getX() / 2, -frameSize.getY() / 2), can_drive_rf, enc_drive_rf, can_steer_rf, enc_steer_rf, steeringEncoderMultiplier, m_floatTolerance, false, false, 0.0)
  );

  // DifferentialDriveModule diffDriveModule = new DifferentialDriveModule("differentialDrive", can_steer_rr, can_drive_lf);

  ModuleController modules;

  Hashtable<String, AutoController> autoModes = new Hashtable<String, AutoController>();
  AutoController currentAutoMode;

  private DoubleSubscriber slider0Sub;

  public Robot() {
    gc_timer.start();

    // TODO 1: figure out if we actually need this - clogs up the log - for raspi?
    // CameraServer.startAutomaticCapture();
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
    var smartDash = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    slider0Sub= smartDash.getDoubleTopic("DB/Slider 0").subscribe(1.0);

    // initialize game controller first because other classes need it.
    // can't initialize in declaration because of order of initialization issues
    m_controller = GameController.Initialize();

    modules = new ModuleController(swerveDriveModule, m_divider, m_controller);

    modules.AddModule(elevator);
    modules.AddModule(slide);
    
    modules.Initialize();

    // modules.SetEnableDrive(true);
    // modules.SetEnableSteer(true);
    // modules.SetEnableDriveTrain(false);

    Dashboard.InitializeChoosers();

    autoModes = AutoModes.Initialize(autoModes, m_controller, modules);

    currentAutoMode = AutoModes.GetDefault(autoModes);
  }

  void commonInit() {
    var startPadding = (startArea.getX() - frameSize.getX()) / 2.0;
    var fieldCenter = new Translation2d(fieldSize.getX() / 2.0, fieldSize.getY() / 2.0);
    var robotCenter = new Translation2d(frameSize.getX() / 2.0, frameSize.getY() / 2.0);
    var robotOffset = robotCenter.plus(new Translation2d(startPadding, startPadding * 2));

    var blueStartPositions = new Translation2d[] {
      new Translation2d(fieldCenter.getX() - startArea.getX() + robotOffset.getX(), fieldSize.getY() - (robotOffset.getY() * 1)),
      new Translation2d(fieldCenter.getX() - startArea.getX() + robotOffset.getX(), fieldSize.getY() - (robotOffset.getY() * 2)),
      new Translation2d(fieldCenter.getX() - startArea.getX() + robotOffset.getX(), fieldSize.getY() - (robotOffset.getY() * 3))
    };

    var driverLocation = 1;
    if (DriverStation.getLocation().isPresent())
      driverLocation = DriverStation.getLocation().getAsInt();

    var blueStartPosition = blueStartPositions[driverLocation - 1];
    var blueStartPose = new Pose3d(new Translation3d(blueStartPosition), Rotation3d.kZero);
    var redStartTransform = new Transform3d(new Translation3d(fieldCenter), new Rotation3d(new Rotation2d(Math.PI)));
    System.out.printf("Robot Init: Driver Location %d, Red Alliance %b, Start Pos (%.2f, %.2f)\n", driverLocation, Utility.IsRedAlliance(), blueStartPosition.getX(), blueStartPosition.getY());

    // Add action poses before button mappings so buttons can drive action poses
    ActionPoses.Initialize(redStartTransform, swerveDriveModule, elevator, slide);

    // even tho this runs on every init, it only happens once so we don't mess up
    Joystick.InitializeButtonMappings(m_controller, modules, swerveDriveModule, slide, elevator); //, grabber);

    if (!initialized) { // only set start position once per match
      initialized = true;
      // real robot starts at (0,0) so that we know we don't have a vision estimate yet.
      Pose3d startPose = Pose3d.kZero;

      if (Robot.isSimulation()) {
        startPose = blueStartPose;
        if (Utility.IsRedAlliance()) {
          startPose = new Pose3d(startPose.rotateAround(redStartTransform.getTranslation(), redStartTransform.getRotation()).getTranslation(), Rotation3d.kZero);
        }
      }

      modules.GetDriveModule().SetCurrentPose(startPose);
    }
  }

  void commonPeriodic() {
    // run the garbage collector every 5 seconds
    if (gc_timer.advanceIfElapsed(5))
      System.gc();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    commonInit();

    var selectedMode = currentAutoMode;
    if (!isSimulation()) {
      var selectedValue = SmartDashboard.getString("Auto Selector", autoModes.keys().nextElement());
      System.out.printf("selected auto value '%s'\n", selectedValue);
      if (selectedValue != null)
        selectedMode = autoModes.get(selectedValue);
    }
    System.out.printf("selected auto mode '%s'\n", selectedMode.GetLabel());
    selectedMode.Initialize();

    // default to field oriented for Auto
    modules.GetDriveModule().SetFieldOriented(true);

    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    commonPeriodic();

    currentAutoMode.Update();
    
    modules.ProcessState(true);
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    commonInit();

    // switch back to defined field oriented mode when we start up tele-op; prevents bleedover from auto
    modules.GetDriveModule().SetFieldOriented(isFieldOriented);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    commonPeriodic();

    // get settings from dashboard
    // slider 0 is motor speed
    modules.setSpeedMod(slider0Sub.get());

    modules.ProcessState(false);
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    commonInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    commonPeriodic();
  }
}
