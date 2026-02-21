// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.z2024;

import java.util.Hashtable;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.auto.AutoController;
import frc.robot.gyro.AHRSGyro;
import frc.robot.gyro.Gyro;
import frc.robot.misc.GameController;
import frc.robot.misc.Utility;
import frc.robot.modules.ModuleController;
import frc.robot.modules.SingleMotorModule;
import frc.robot.modules.DifferentialDriveModule;
import frc.robot.modules.DualMotorModule;
import frc.robot.positioner.LimeLightPositioner;
import frc.robot.positioner.Positioner;
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

  double elapsedTime;

  final String codeBuildVersion = "2025.10.18-THOR";
  boolean initialized = false;

  final PWMTalonSRX can_drive_left = new PWMTalonSRX(5);
  final PWMTalonSRX can_drive_right = new PWMTalonSRX(4);

  final Spark can_shoot_left = new Spark(8);
  final Spark can_shoot_right = new Spark(9);
  final Spark can_pickup = new Spark(1);
  final Spark can_feed = new Spark(0);
  final Spark can_lift_left = new Spark(6);
  final Spark can_lift_right = new Spark(7);

  final Encoder enc_drive_left = null; // new REVEncoder(can_drive_left.getEncoder());
  final Encoder enc_drive_right = null; // new REVEncoder(can_drive_right.getEncoder());

  final Encoder enc_shoot_left = null; // new REVEncoder(can_shoot_left.getEncoder());
  final Encoder enc_shoot_right = null; // new REVEncoder(can_shoot_right.getEncoder());

  // final Encoder enc_lift_left = new REVEncoder(can_lift_left.getEncoder());
  // final Encoder enc_lift_right = new REVEncoder(can_lift_right.getEncoder());

  final Gyro m_gyro = new AHRSGyro();
  final Positioner m_positioner = new LimeLightPositioner(true);

  GameController m_controller = null;

  final Timer m_timer = new Timer();
  final Timer gc_timer = new Timer();

  final boolean isFieldOriented = true;
  SingleMotorModule shoot_left = new SingleMotorModule("shoot_left", can_shoot_left, Constants.shootSpeed, false, null, null, enc_shoot_left, 1.0, 1.0, Constants.shootDistancePerRotation, 0.0, true);
  SingleMotorModule shoot_right = new SingleMotorModule("shoot_right", can_shoot_right, Constants.shootSpeed, true, null, null, enc_shoot_right, 1.0, 1.0, Constants.shootDistancePerRotation, 0.0, true);
  DualMotorModule shoot = new DualMotorModule("shoot", shoot_left, shoot_right);

  SingleMotorModule lift_left = new SingleMotorModule("lift_left", can_lift_left, Constants.liftSpeed, false, null, null, null, 1.0, 1.0, Constants.liftDistancePerRotation, 0.0, false);
  SingleMotorModule lift_right = new SingleMotorModule("lift_right", can_lift_right, Constants.liftSpeed, true, null, null, null, 1.0, 1.0, Constants.liftDistancePerRotation, 0.0, false);
  DualMotorModule lift = new DualMotorModule("lift", lift_left, lift_right);

  SingleMotorModule feed = new SingleMotorModule("feed", can_feed, Constants.feedSpeed, false, null, null, null, 1.0, 1.0, Constants.feedDistancePerRotation, 0.0, true);
  SingleMotorModule pickup = new SingleMotorModule("pickup", can_pickup, Constants.pickupSpeed, false, null, null, null, 1.0, 1.0, Constants.pickupDistancePerRotation, 0.0, true);

  DifferentialDriveModule diffDrive = new DifferentialDriveModule("diffDrive", m_gyro, m_positioner, Constants.driveSpeed, can_drive_left, enc_drive_left, can_drive_right, enc_drive_right, Constants.driveRatio, Constants.floatTolerance, Constants.frameSize.getNorm(), Constants.robotSize.getY());

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
    Utility.setFieldSize(Constants.fieldSize);
    // rotate around z axis to localize the x coordinates, rotate around the x axis to localize the y coordinates
    Utility.setRedStartTransform(new Transform3d(new Translation3d(Constants.fieldCenter), new Rotation3d(Math.PI, 0, Math.PI)));

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

    modules = new ModuleController(diffDrive, Constants.divider);

    modules.AddModule(shoot);
    modules.AddModule(feed);
    modules.AddModule(pickup);
    modules.AddModule(lift);

    // initialize modules
    modules.Initialize();

    // modules.SetEnableDrive(true);
    // modules.SetEnableSteer(true);
    // modules.SetEnableDriveTrain(false);

    autoModes = AutoModes.Initialize(autoModes, modules);
    currentAutoMode = AutoModes.GetDefault(autoModes);
  }

  void commonInit() {
    // initialize game controller first because other classes need it.
    // This needs to be here in mode init because we may not have a driver station connection during robotinit.
    m_controller = GameController.Initialize();
    // Add action poses before button mappings so buttons can drive action poses
    ActionPoses.Initialize(diffDrive, shoot, feed, pickup, lift);
    // even tho this runs on every init, we clear it out before every run so we don't mess up
    Joystick.InitializeButtonMappings(m_controller, modules, diffDrive, shoot, feed, pickup, lift);

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
