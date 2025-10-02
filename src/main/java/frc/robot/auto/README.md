# Autonomous Framework (FRC3680)

Overview
- AutoController: manages one or more AutoSequence instances.
- AutoSequence: ordered steps (AutoEvent) with triggers: Target, Time, Position, Auto.
- Controller-target flow: broadcast a single “global” ActionPose via ModuleController; each module resolves what it should do for that intent.
- Modules publish their own local ActionPoses keyed by the same categorical fields; the Pose3d payload is module-specific.

Lifecycle (abridged)
- Register: create AutoController, add sequences, publish names to SmartDashboard.
- Autonomous:
  - autonomousInit: pick from dashboard, Initialize().
  - autonomousPeriodic: currentAutoMode.Update(); modules.ProcessState(true).

Controller-target pattern (recommended)
A single ActionPose expresses the intent for the robot across all subsystems. The ModuleController broadcasts it to every module; each module looks up its own local pose for that same categorical key and drives itself to completion, then clears its target.

ActionPose fields
- group (Group): high-level phase (e.g., Score, Pickup).
- location (Location): field element (e.g., Reef, Coral).
- locationIndex (int): which instance (0..N) of that field element.
- position (Position): sub-position at that element (e.g., Lower/L2/Trough).
- action (Action): operation being performed (e.g., Drop, Pickup).
- pose (Pose3d): module-specific payload when defined inside a module’s local library.
  - SwerveDriveModule: field Translation3d + heading (Rotation3d.z) to drive/rotate to.
  - SingleMotorModule (elevator): pose.x is the motor-axis target (rotations/units after its encoderMultiplier); y/z ignored.
  - SingleActuatorModule (relay/slide): pose.x > 0 = forward/open, < 0 = reverse/close, 0 = off; module holds for holdTime then clears.

How modules evaluate ActionPoses
- Each module registers its local library with AddActionPose(...). The library entries use the same categorical keys (group, location, locationIndex, position, action) but carry that module’s Pose3d semantics.
- When the controller broadcasts SetTargetActionPose(ActionPose), the module resolves a local match with GetActionPose(...) using “exact-or-Any” matching on each categorical field.
- If found, the module sets an internal target and self-drives until done, then AbandonTarget() to signal completion.

Defining holistic ActionPoses (robotInit)
Define a consistent set of categorical keys across modules; each module adds its own local Pose3d for the same keys.

Example: one intent, many modules
```java
// Swerve (field pose and heading for that scoring bay)
swerveDriveModule.AddActionPose(
  new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Drop,
                 new Pose3d(/* field X,Y,Z */, new Rotation3d(0, 0, /* headingRad */))));

// Elevator (height in local motor units via pose.x)
elevator.AddActionPose(
  new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Drop,
                 new Pose3d(/* rotations on X */ 1.14, 0, 0, new Rotation3d())));

// Slide (open during drop; positive X = forward/open)
slide.AddActionPose(
  new ActionPose(Group.Score, Location.Reef, 0, Position.Middle, Action.Drop,
                 new Pose3d(1, 0, 0, new Rotation3d())));
```

Sequencing with controller-target
Use AutoEventTarget with moduleController to broadcast global intents; AwaitTarget waits until all modules clear their targets.

```java
public class SeqControllerTargetDemo extends AutoSequence {
  public SeqControllerTargetDemo(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    // One global intent: Score at Reef[0], L2, Drop
    var scoreL2Drop = new ActionPose(
      Group.Score, Location.Reef, 0, Position.Middle, Action.Drop, new Pose3d());

    // Broadcast intent (SetTarget); modules resolve their local poses
    AutoEventTarget setIntent = new AutoEventTarget("Set Score L2 Drop", true, scoreL2Drop, AutoEvent.EventType.SetTarget, ac);
    setIntent.moduleController = modules;
    AddEvent(setIntent);

    // Await all modules to finish (each AbandonTarget() when done)
    AutoEventTarget awaitAll = new AutoEventTarget("Await All", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitAll.moduleController = modules;
    AddEvent(awaitAll);
  }
}
```

Distinguishing field actions with ActionPose keys
- Group separates phases (Score vs Pickup).
- Location and locationIndex select which field element/bay.
- Position distinguishes sub-levels or L1/L2/L3 equivalent.
- Action captures the specific operation at that point (Drop vs Pickup).
- Pose3d is module-local data (units/semantics differ per module) but shares the same categorical keys for alignment.

Notes
- Units: meters/radians in Pose3d; modules may interpret pose.x as motor-units (SingleMotorModule) or direction (SingleActuatorModule).
- SwerveDriveModule will not drive if positioner health fails (NaN/inf/large jumps/stale); it clears target only when all PID goals are reached.
- Ensure every module that should act on an intent has a matching AddActionPose entry for those categorical keys; otherwise GetActionPose returns null and the module ignores that intent.

## Module-target approach (per-module targets)
When you need to direct a specific module (or stagger actions independently), use per-module SetTarget/AwaitTarget. This is also useful for testing modules in isolation.

Key points
- targetModule points to a specific RobotModule.
- SetTarget assigns that module’s target and completes immediately.
- AwaitTarget completes when that module clears its target (GetTarget() == null).

Example
```java
public class SeqModuleTargetDemo extends AutoSequence {
  public SeqModuleTargetDemo(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    var drive = (frc.robot.SwerveDriveModule) modules.GetDriveModule();
    var elevator = (frc.robot.SingleMotorModule) modules.GetModule("elevator");

    // Prepare or fetch module-local poses
    var rotate90 = /* ActionPose previously added to drive */;
    var elevL2 = elevator.GetActionPose(
      frc.robot.action.Group.Score, frc.robot.action.Location.Any, -1,
      frc.robot.action.Position.Middle, frc.robot.action.Action.Any);

    // Parallel: drive rotate + elevator to L2
    AutoEventTarget setDrive = new AutoEventTarget("Set Drive 90", true, rotate90, AutoEvent.EventType.SetTarget, ac);
    setDrive.targetModule = drive;
    AddEvent(setDrive);

    AutoEventTarget setElev = new AutoEventTarget("Set Elevator L2", true, elevL2, AutoEvent.EventType.SetTarget, ac);
    setElev.targetModule = elevator;
    AddEvent(setElev);

    // Await both
    AutoEventTarget awaitDrive = new AutoEventTarget("Await Drive", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitDrive.targetModule = drive;
    AddEvent(awaitDrive);

    AutoEventTarget awaitElev = new AutoEventTarget("Await Elevator", false, null, AutoEvent.EventType.AwaitTarget, ac);
    awaitElev.targetModule = elevator;
    AddEvent(awaitElev);
  }
}
```

Tips
- Prefer controller-target for holistic routines. Module-target is great for overrides, debugging, or phased starts/stops per module.
- Ensure each module has a matching AddActionPose for the requested keys; otherwise GetActionPose(...) returns null and the module will ignore the target.

## Time-based approach (open-loop holds)
Use timed events when you must apply a raw command for a duration (e.g., relays, open-loop test movement). Time events call Run() every loop while elapsed < milliseconds.

Key points
- 0 ms does not call Run(); it completes immediately. Use ms > 0 for any effect.
- Boolean and Double events are applied each loop during the hold window.
- Combine with short “stop” pulses when needed.

Example
```java
public class SeqTimeBasedDemo extends AutoSequence {
  public SeqTimeBasedDemo(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    var drive = modules.GetDriveModule();
    var slide = (frc.robot.SingleActuatorModule) modules.GetModule("slide");

    // Drive forward for 2 seconds
    AutoEventTime fwd2s = new AutoEventTime("Drive Fwd 2s", false, 2000, AutoEvent.EventType.Double, ac);
    fwd2s.doubleEvent = drive::ProcessForwardSpeed;
    fwd2s.doubleValue = 0.5;
    AddEvent(fwd2s);

    // Stop (short non-zero duration so Run() executes)
    AutoEventTime stop = new AutoEventTime("Stop Drive", false, 50, AutoEvent.EventType.Double, ac);
    stop.doubleEvent = drive::ProcessForwardSpeed;
    stop.doubleValue = 0.0;
    AddEvent(stop);

    // Open latch for 2 seconds, then close briefly
    AutoEventTime open = new AutoEventTime("Open Latch 2s", true, 2000, AutoEvent.EventType.Boolean, ac);
    open.boolEvent = slide::ApplyValue;
    open.boolValue = true;
    AddEvent(open);

    AutoEventTime close = new AutoEventTime("Close Latch", false, 50, AutoEvent.EventType.Boolean, ac);
    close.boolEvent = slide::ApplyInverse;
    close.boolValue = true;
    AddEvent(close);
  }
}
```

When to use
- Quick subsystem tests or simple autonomous behaviors that don’t require closed-loop completion.
- Actuators that must be asserted continuously to stay active (Relay-based, etc.).
- As a complement to controller- or module-target sequences when you need timed interlocks or delays.

Integration notes
- Time-based holds run concurrently with controller/module targets when events are marked parallel=true.
- Modules like SingleActuatorModule reset to kOff each ProcessState; time-based holds continuously reapply the requested state while the event is active.

Legacy/time-based patterns
- Module-target: SetTarget/AwaitTarget on a specific module remains supported but is less holistic.
- Time-based: AutoEventTime(Boolean/Double) applies outputs each loop while elapsed < ms. Use ms > 0; 0 ms completes without calling Run().
