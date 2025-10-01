package frc.robot.auto;

import frc.robot.ModuleController;
import frc.robot.DriveModule;
import frc.robot.SingleActuatorModule;

// SequencePositionDemo is an example autonomous sequence that demonstrates using a position-based event to control a module.
public class SequencePositionDemo extends AutoSequence {
  public SequencePositionDemo(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    // Modules
    DriveModule drive = modules.GetDriveModule();
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    // Target: current pose (event will be "near" immediately; holds while near)
    var startPose = drive.GetPosition();

    // Parallel Position event: keep latch open while near the start pose.
    // Completes when we leave the nearby region.
    AutoEventPosition holdOpenNearStart = new AutoEventPosition(
      "Hold Open Near Start", true, startPose, AutoEvent.EventType.Boolean, ac, drive
    );
    holdOpenNearStart.boolEvent = slide::ApplyValue;
    holdOpenNearStart.boolValue = true;
    AddEvent(holdOpenNearStart);

    // Drive forward for 2 seconds so we leave the start pose region.
    AutoEventTime driveForward = new AutoEventTime("Drive Forward 2s", false, 2000, AutoEvent.EventType.Double, ac);
    driveForward.doubleEvent = drive::ProcessForwardSpeed;
    driveForward.doubleValue = 0.5;
    AddEvent(driveForward);

    // Stop the drive (use non-zero duration so Run() executes).
    AutoEventTime stopDrive = new AutoEventTime("Stop Drive", false, 50, AutoEvent.EventType.Double, ac);
    stopDrive.doubleEvent = drive::ProcessForwardSpeed;
    stopDrive.doubleValue = 0.0;
    AddEvent(stopDrive);

    // Close the latch (short hold so ApplyInverse is actually applied).
    AutoEventTime closeLatch = new AutoEventTime("Close Latch", false, 50, AutoEvent.EventType.Boolean, ac);
    closeLatch.boolEvent = slide::ApplyInverse;
    closeLatch.boolValue = true;
    AddEvent(closeLatch);
  }
}
