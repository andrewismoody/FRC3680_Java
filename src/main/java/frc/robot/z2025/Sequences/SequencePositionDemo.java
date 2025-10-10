package frc.robot.z2025.Sequences;

import frc.robot.auto.AutoController;
import frc.robot.auto.AutoEvent;
import frc.robot.auto.AutoEventPosition;
import frc.robot.auto.AutoEventTime;
import frc.robot.auto.AutoSequence;
import frc.robot.modules.DriveModule;
import frc.robot.modules.SingleActuatorModule;

// SequencePositionDemo is an example autonomous sequence that demonstrates using a position-based event to control a module.
public class SequencePositionDemo extends AutoSequence {
  private final AutoController autoController;
  private boolean initialized = false;

  public SequencePositionDemo(String label, AutoController ac) {
    super(label, ac);
    autoController = ac;
  }

  @Override
  public void Initialize() {
    super.Initialize();

    if (initialized) {
      System.out.printf("Sequence %s already initialized; skipping duplicate init\n", GetLabel());
      return;
    }
    initialized = true;

    var modules = autoController.GetModuleController();

    // Modules
    DriveModule drive = modules.GetDriveModule();
    var slide = (SingleActuatorModule) modules.GetModule("slide");

    // Target: current pose (event will be "near" immediately; holds while near)
    var startPose = drive.GetPosition();

    // Parallel Position event: keep latch open while near the start pose.
    // Completes when we leave the nearby region.
    AutoEventPosition holdOpenNearStart = new AutoEventPosition(
      "Hold Open Near Start", true, startPose, AutoEvent.EventType.Boolean, autoController, drive
    );
    holdOpenNearStart.SetBoolEvent(true, slide::ApplyValue);
    AddEvent(holdOpenNearStart);

    // Drive forward for 2 seconds so we leave the start pose region.
    AutoEventTime driveForward = new AutoEventTime("Drive Forward 2s", false, 2000, AutoEvent.EventType.Double, autoController);
    driveForward.SetDoubleEvent(0.5, drive::ProcessForwardSpeed);
    AddEvent(driveForward);

    // Stop the drive (use non-zero duration so Run() executes).
    AutoEventTime stopDrive = new AutoEventTime("Stop Drive", false, 50, AutoEvent.EventType.Double, autoController);
    stopDrive.SetDoubleEvent(0.0, drive::ProcessForwardSpeed);
    AddEvent(stopDrive);

    // Close the latch (short hold so ApplyInverse is actually applied).
    AutoEventTime closeLatch = new AutoEventTime("Close Latch", false, 50, AutoEvent.EventType.Boolean, autoController);
    closeLatch.SetBoolEvent(true, slide::ApplyInverse);
    AddEvent(closeLatch);
  }
}
