// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.z2025;

import java.util.Hashtable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.modules.SwerveMotorDefinition;
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

  final String codeBuildVersion = "2025.10.18-THOR";
  boolean initialized = false;

  final SparkMax can_drive_lf = new SparkMax(4, MotorType.kBrushless);
  final SparkMax can_steer_lf = new SparkMax(5,MotorType.kBrushless);

  final SparkMax can_drive_rf = new SparkMax(6, MotorType.kBrushless);
  final SparkMax can_steer_rf = new SparkMax(7, MotorType.kBrushless);

  final SparkMax can_drive_lr = new SparkMax(8, MotorType.kBrushless);
  final SparkMax can_steer_lr = new SparkMax(9, MotorType.kBrushless);

  final SparkMax can_drive_rr = new SparkMax(10, MotorType.kBrushless);
  final SparkMax can_steer_rr = new SparkMax(3, MotorType.kBrushless);

  final SparkMax can_elev = new SparkMax(2, MotorType.kBrushless);

  final Relay pwm_slide = new Relay(0);

  final Encoder enc_steer_lf = new REVEncoder(can_steer_lf.getEncoder());
  final Encoder enc_steer_rf = new REVEncoder(can_steer_rf.getEncoder());
  final Encoder enc_steer_lr = new REVEncoder(can_steer_lr.getEncoder());
  final Encoder enc_steer_rr = new REVEncoder(can_steer_rr.getEncoder());

  final Encoder enc_drive_lf = new REVEncoder(can_drive_lf.getEncoder());
  final Encoder enc_drive_rf = new REVEncoder(can_drive_rf.getEncoder());
  final Encoder enc_drive_lr = new REVEncoder(can_drive_lr.getEncoder());
  final Encoder enc_drive_rr = new REVEncoder(can_drive_rr.getEncoder());

  final Gyro m_gyro = new AHRSGyro();
  final Encoder enc_elev = new REVEncoder(can_elev.getEncoder());
  final Positioner m_positioner = new LimeLightPositioner(true);

  GameController m_controller = null;

  final Timer m_timer = new Timer();
  final Timer gc_timer = new Timer();

  final boolean isFieldOriented = true;
  SingleMotorModule elevator = new SingleMotorModule("elevator", can_elev, Constants.elevatorSpeed, false, null, null, enc_elev, Constants.elevatorEncoderMultiplier, 0.5, Constants.elevatorDistancePerRotation, Constants.elevatorMaxDistance);
  SingleActuatorModule slide = new SingleActuatorModule("slide", pwm_slide, false);

  // leftFront  software position // potentially should be leftrear   hardware position
  SwerveMotorDefinition leftFrontDef = new SwerveMotorDefinition(can_drive_lf, enc_drive_lf, can_steer_lf, enc_steer_lf);
  // rightFront software position // potentially should be leftFront  hardware position
  SwerveMotorDefinition rightFrontDef = new SwerveMotorDefinition(can_drive_rf, enc_drive_rf, can_steer_rf, enc_steer_rf);
  // leftRear   software position // potentially should be rightRear  hardware position
  SwerveMotorDefinition leftRearDef = new SwerveMotorDefinition(can_drive_lr, enc_drive_lr, can_steer_lr, enc_steer_lr);
  // rightRear  software position // potentially should be rightFront hardware position
  SwerveMotorDefinition rightRearDef = new SwerveMotorDefinition(can_drive_rr, enc_drive_rr, can_steer_rr, enc_steer_rr);
  // total length of robot is 32.375", width is 27.5", centerline is 16.1875" from edge.  Drive axle center is 4" from edge - 12.1875" from center which is 309.56mm or 0.30956 meters
  // motor positions are rotated to make the limelight 'forward', this is just labeling.
  SwerveDriveModule swerveDriveModule = new SwerveDriveModule("swerveDrive", m_gyro, m_positioner, Constants.driveSpeed, Constants.driveRatio, Constants.steerMotorSpeed, Constants.floatTolerance
    , new SwerveMotorModule(SwervePosition.LeftFront, new Translation2d(Constants.motorPosition.getX(), Constants.motorPosition.getY()), rightFrontDef, Constants.steeringEncoderMultiplier, Constants.floatTolerance, true, false, 0.0)
    , new SwerveMotorModule(SwervePosition.RightFront, new Translation2d(Constants.motorPosition.getX(), -Constants.motorPosition.getY()), rightRearDef, Constants.steeringEncoderMultiplier, Constants.floatTolerance, true, false, 0.0)
    , new SwerveMotorModule(SwervePosition.LeftRear, new Translation2d(-Constants.motorPosition.getX(), Constants.motorPosition.getY()), leftFrontDef, Constants.steeringEncoderMultiplier, Constants.floatTolerance, true, false, 0.0)
    , new SwerveMotorModule(SwervePosition.RightRear, new Translation2d(-Constants.motorPosition.getX(), -Constants.motorPosition.getY()), leftRearDef, Constants.steeringEncoderMultiplier, Constants.floatTolerance, true, false, 0.0)
  );

  ModuleController modules;

  Hashtable<String, AutoController> autoModes = new Hashtable<String, AutoController>();
  AutoController currentAutoMode;

  private DoubleSubscriber slider0Sub;

  public Robot() {
    gc_timer.start();
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Utility.fieldSize = Constants.fieldSize;
    SmartDashboard.putString("DB/String 0", "Code Build Version: ");
    SmartDashboard.putString("DB/String 5", codeBuildVersion);
    SmartDashboard.putString("DB/String 1", "Start Location: "); // this is actual start position; if zero, read from driver station
    if (RobotBase.isReal())
      SmartDashboard.putString("DB/String 6", "0"); // this is actual start position; if zero, read from driver station
    else {
      if (SmartDashboard.getString("DB/String 6", "-1") == "-1")
        SmartDashboard.putString("DB/String 6", "0"); // this is actual start position; if zero, read from driver station
    }
    SmartDashboard.putNumber("DB/Slider 0", Constants.speedMod);
    var smartDash = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    slider0Sub= smartDash.getDoubleTopic("DB/Slider 0").subscribe(1.0);

    modules = new ModuleController(swerveDriveModule, Constants.divider);

    modules.AddModule(elevator);
    modules.AddModule(slide);
    
    // initialize modules
    modules.Initialize();

    // modules.SetEnableDrive(true);
    // modules.SetEnableSteer(true);
    // modules.SetEnableDriveTrain(false);

    Dashboard.InitializeChoosers();
    autoModes = AutoModes.Initialize(autoModes, modules);
    currentAutoMode = AutoModes.GetDefault(autoModes);
  }

  void commonInit() {
    // initialize game controller first because other classes need it.
    // This needs to be here in mode init because we may not have a driver station connection during robotinit.
    m_controller = GameController.Initialize();
    // Add action poses before button mappings so buttons can drive action poses
    ActionPoses.Initialize(swerveDriveModule, elevator, slide);
    // even tho this runs on every init, we clear it out before every run so we don't mess up
    Joystick.InitializeButtonMappings(m_controller, modules, swerveDriveModule, slide, elevator);

    // only set start position once per match
    if (!initialized) { 
      initialized = true;

      // real robot starts at (0,0) so that we know we don't have a vision estimate yet.
      Pose3d startPose = Pose3d.kZero;

      startPose = Constants.getMyStartPose();
      System.out.printf("Robot Init: Driver Location %d, Red Alliance %b, Start Pos (%.2f, %.2f)\n", Utility.getDriverLocation(), Utility.IsRedAlliance(), startPose.getX(), startPose.getY());

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
    // swerveDriveModule.ResetGyro();
    // swerveDriveModule.ResetEncoders();

    var selectedMode = currentAutoMode;
    var selectedValue = SmartDashboard.getString("Auto Selector", currentAutoMode.GetLabel());
    System.out.printf("selected auto value '%s'\n", selectedValue);
    if (selectedValue != null) {
      selectedMode = autoModes.get(selectedValue);
      System.out.printf("selected auto mode '%s'\n", selectedMode.GetLabel());
    }
    currentAutoMode = selectedMode;
    selectedMode.Initialize(m_controller);

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

    m_controller.ProcessButtons();
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
