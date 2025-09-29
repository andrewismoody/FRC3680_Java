# Autonomous System Documentation

This folder implements autonomous using AutoSequences (ordered lists of AutoEvents). Events are triggered by Time, Position, or Auto, and can run in parallel or sequentially.

---

## Event Types

- Time
  - Use AutoEventTime to trigger Void/Boolean/Double/Auto actions after a delay.
- Position (Adaptive pattern)
  - SetTarget: tell a module to pursue a target; event completes immediately.
  - AwaitTarget: wait until the module reports it has reached/cleared its target; event completes when module clears the target.
- Auto
  - Queue or monitor nested sequences.

Note: The “adaptive” behavior is achieved via Position events using SetTarget and AwaitTarget. Modules decide when they are “finished” by clearing their target; events only mark “complete” when appropriate.

---

## Module Feedback Contract

- SwerveDriveModule
  - SetTargetActionPose(ActionPose)
  - GetTarget(): ActionPose or null when done
  - Internally clears target when reached.
- SingleMotorModule
  - SetTargetActionPose(...), GetTarget()
  - Clears target when the goal is reached.

AutoEventPosition ties these together:
- SetTarget: targetModule.SetTargetActionPose(target); event completes now.
- AwaitTarget: if targetModule.GetTarget() == null then event completes; otherwise keep waiting.

AutoSequence.Update drives this by calling Run() on events when their trigger condition is met.

---

## Minimal Examples

### Timed Shooter (matches SequenceShoot)
```java
public class SequenceShoot extends AutoSequence {
    public SequenceShoot(String label, ModuleController modules, AutoController ac) {
        super(label, modules, ac);
        var ejector = modules.GetModule("ejector");
        if (ejector != null) {
            AutoEventTime start = new AutoEventTime("Start Shooter", false, 0, EventType.Boolean, ac);
            start.boolEvent = ejector::ProcessState;
            start.boolValue = true;
            AddEvent(start);

            AutoEventTime stop = new AutoEventTime("Stop Shooter", false, 2000, EventType.Boolean, ac);
            stop.boolEvent = ejector::ProcessState;
            stop.boolValue = false;
            AddEvent(stop);
        }
    }
}
```

### Drive to Pose (Position: SetTarget + AwaitTarget)
```java
// Prepare an ActionPose in your Drive module at init time (example)
swerveDriveModule.AddActionPose(
  new ActionPose(Group.Score, Location.Any, -1, Position.Lower,
                 Action.Any, new Pose3d(/* x,y,z + rot */)));

// In a sequence, first set the target, then await completion
AutoEventPosition set = new AutoEventPosition(
  "Set Drive Target", false,
  swerveDriveModule.GetActionPose(Group.Score, Location.Any, -1, Position.Lower, Action.Any),
  EventType.SetTarget, autoController);
set.targetModule = swerveDriveModule;
AddEvent(set);

AutoEventPosition await = new AutoEventPosition(
  "Await Drive Target", false, null,
  EventType.AwaitTarget, autoController);
await.targetModule = swerveDriveModule;
AddEvent(await);
```

Tip:
- Use SetTarget for any module that supports SetTargetActionPose/GetTarget (e.g., SwerveDriveModule, SingleMotorModule).
- Always follow SetTarget with AwaitTarget if you need to block until the module finishes.

---

## Sequence Examples

### Move and Stop (matches SequenceMoveAndShoot structure)
```java
public class SequenceMoveAndStop extends AutoSequence {
  public SequenceMoveAndStop(String label, ModuleController modules, AutoController ac) {
    super(label, modules, ac);

    AutoEventTime startDrive = new AutoEventTime("Move Forward", true, 0, EventType.Double, ac);
    startDrive.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
    startDrive.doubleValue = 1.0;
    AddEvent(startDrive);

    AutoEventTime stopDrive = new AutoEventTime("Stop Driving", false, 3000, EventType.Double, ac);
    stopDrive.doubleEvent = modules.GetDriveModule()::ProcessForwardSpeed;
    stopDrive.doubleValue = 0.0;
    AddEvent(stopDrive);
  }
}
```

---

## Driver Station Selection

Option A: SendableChooser (recommended)
```java
// In Robot.robotInit()
var chooser = new edu.wpi.first.wpilibj.smartdashboard.SendableChooser<String>();
AutoController moveShoot = new AutoController("MoveAndShoot");
// moveShoot.AddSequence(new SequenceMoveAndShoot(moveShoot.GetLabel(), modules, moveShoot));
chooser.setDefaultOption(moveShoot.GetLabel(), moveShoot.GetLabel());
SmartDashboard.putData("Auto Selector", chooser);

// Keep a map of modes
java.util.Hashtable<String, AutoController> AutoModes = new java.util.Hashtable<>();
AutoModes.put(moveShoot.GetLabel(), moveShoot);
```

Option B: String array (matches current Robot.java)
```java
// In Robot.robotInit()
AutoController timedShoot = new AutoController("MoveAndShoot");
// timedShoot.AddSequence(new SequenceMoveAndShoot(timedShoot.GetLabel(), modules, timedShoot));
AutoModes.put(timedShoot.GetLabel(), timedShoot);
SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));
```

Selection on autonomousInit (chooser or string key):
```java
// Using chooser:
var data = SmartDashboard.getData("Auto Selector");
String selected = data instanceof edu.wpi.first.wpilibj.smartdashboard.SendableChooser
  ? ((edu.wpi.first.wpilibj.smartdashboard.SendableChooser<String>) data).getSelected()
  : null;

// Using string array:
String[] list = SmartDashboard.getStringArray("Auto List", new String[] {});
String fallback = list.length > 0 ? list[0] : null;
selected = selected != null ? selected : SmartDashboard.getString("Auto Selector", fallback);

// Resolve and start:
AutoController currentAuto = selected != null ? AutoModes.get(selected) : null;
if (currentAuto != null) currentAuto.Initialize();
```

---

## Notes

- AutoSequence.Update already handles:
  - Time trigger: compares elapsed to event.milliseconds.
  - Position trigger:
    - SetTarget runs once and completes.
    - AwaitTarget polls module.GetTarget() until null, then completes.
  - Auto trigger: use for nested sequences.
- Deprecated proximity helper isNearby() exists but SetTarget/AwaitTarget is preferred.

---

## Troubleshooting

- Event never completes: ensure AwaitTarget has targetModule set and the module clears its target.
- Module doesn’t move: verify ActionPose exists in the module (AddActionPose) and SetTargetActionPose resolves a non-null pose.
- DS selection empty: ensure AutoModes is populated and SmartDashboard is updated in robotInit.
