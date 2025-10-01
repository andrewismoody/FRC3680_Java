# Autonomous Framework (FRC3680)

Overview
- AutoController: manages one or more AutoSequence instances.
- AutoSequence: ordered list of AutoEvent steps.
- AutoEvent: concrete event types (Time, Target, Position, Auto) with Run().

Lifecycle and selection
- Register:
  ```java
  // ...existing code...
  AutoController rotateWait = new AutoController("RotateWait");
  rotateWait.AddSequence(new SequenceRotateWaitReturn(rotateWait.GetLabel(), modules, rotateWait));
  AutoModes.put(rotateWait.GetLabel(), rotateWait);
  SmartDashboard.putStringArray("Auto List", AutoModes.keySet().toArray(new String[] {}));
  // ...existing code...
  ```
- Initialize and run:
  ```java
  // autonomousInit
  currentAutoMode = AutoModes.get(SmartDashboard.getString("Auto Selector", AutoModes.keys().nextElement()));
  currentAutoMode.Initialize();

  // autonomousPeriodic
  currentAutoMode.Update();
  modules.ProcessState(true);
  ```

Recent changes
- TriggerType.Target with AutoEventTarget
  - EventType.SetTarget: dispatch an ActionPose to a module (completes immediately).
  - EventType.AwaitTarget: completes when module clears its target (e.g., AbandonTarget()).
- Legacy TriggerType.Position still supported for backward compatibility.
- Time events now “hold until duration”
  - AutoSequence.Update() calls Run() every loop while elapsedTime < milliseconds, then completes and resets startTime.
  - 0 ms Time events do not call Run() (complete immediately).
- SingleActuatorModule supports target-based holds
  - EvaluateTargetPose applies forward/reverse based on pose X (>0 forward, <0 reverse, 0 off).
  - Internal holdTime defaults to 2000 ms before AbandonTarget().

Event types and semantics
- Time
  - Use for Boolean/Double/Void/Auto actions that must be applied across multiple loops for a duration.
  - Note: use milliseconds > 0 if you need Run() to execute.
- Target (preferred for setpoint control)
  - SetTarget: module.SetTargetActionPose(...), completes immediately.
  - AwaitTarget: waits until module.GetTarget() == null.
- Position (legacy)
  - Retained for older flows; prefer Target events for new code.
- Auto
  - Nest and monitor child sequences.

Patterns for modules

1) Target-based setpoints (recommended for motion/position)
- SwerveDriveModule, SingleMotorModule:
  - AddActionPose(...) during robotInit.
  - Sequence uses AutoEventTarget(SetTarget) then AutoEventTarget(AwaitTarget).
- SingleActuatorModule:
  - AddActionPose with X=1 for forward/open, X=-1 for reverse/close, X=0 for off.
  - Use Target events to leverage internal holdTime without extra timers.

2) Direct timed commands (simple on/off or open-loop)
- Time(Boolean, ms) + ApplyValue/ApplyInverse
  - Use ms > 0 to actually apply each loop.
  - Example: hold open for 2000 ms by Time(Boolean, 2000) + slide::ApplyValue.

Example: rotate, score, return (abridged)
- Drive and elevator via Target events:
  - SetTarget: rotate90 and elevL2 (parallel).
  - AwaitTarget: both complete.
- Latch with time or target:
  - Time approach: Time(Boolean, 2000) + slide::ApplyValue; then Time(Boolean, small>0) + slide::ApplyInverse.
  - Target approach: SetTarget to slide pose with X=1 (open); AwaitTarget lets module auto-abandon after holdTime.

ModuleController approaches
- Access:
  - modules.GetDriveModule() -> active DriveModule (swerve/differential).
  - modules.GetModule("elevator" | "slide" | ...) -> specific RobotModule.
- Control styles:
  1) Target-based: SetTargetActionPose + AwaitTarget (module owns completion).
  2) Timed direct: AutoEventTime(Boolean/Double) for sustained application across ms.

Timing details
- AutoSequence.Initialize() captures startTime.
- Any event that completes will reset startTime; following Time events are relative to that reset.

Troubleshooting
- “Latch didn’t open” when using Time(Boolean, 0):
  - 0 ms does not invoke Run(); use duration > 0 ms or use Target-based slide control to leverage holdTime.
- AwaitTarget never completes:
  - Ensure targetModule is set, a valid ActionPose exists (AddActionPose + GetActionPose not null), and the module clears its target.
- Motion doesn’t start:
  - Verify module has the ActionPose you dispatch and that SetTarget is used before AwaitTarget.

Notes
- Poses are in meters/radians (Pose3d/Translation3d/Rotation3d).
- Positioner and Gyro feed GetPosition(); modules should clear targets on completion.
