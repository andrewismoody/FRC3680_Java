# Auto README (2025 refresh)

Overview
- Auto sequences extend AutoSequence and are built in Initialize() (not in constructors).
- Swerve runs radians internally; Positioner expects degrees; gyro is CCW-positive (wrapper negates NavX yaw).
- SwerveDrivePoseEstimator fuses odometry with vision on real hardware; field-oriented is typically enabled for Auto.

Authoring sequences
- Constructor: call super(label, ac) and store AutoController. Do not resolve modules or poses in the constructor.
- Initialize():
  - Call super.Initialize() and guard with a boolean to avoid duplicate setup.
  - Resolve modules via ac.GetModuleController().
  - Build ActionPose targets (Group, Location, Index, Position, Action).
  - Build events and chain:
    - AwaitTarget: new AutoEventTarget(name, isAsync, pose, AutoEvent.EventType.AwaitTarget, ac)
    - Timed events: new AutoEventTime(name, isAsync, durationMs, type, ac)
      - SetBoolEvent(value, consumer) or SetDoubleEvent(value, consumer)
      - For pure waits, use EventType.None
  - Chain with BeginWith(...).Then(...)

Event types and chaining
- AwaitTarget (synchronous): waits until the addressed module(s) report their target reached and settle logic passes.
- Timed Boolean/Double: applies a consumer for the duration; clamped by module implementations.
- None: time-only wait with no consumer side effect.
- Async vs Sync:
  - isAsync=false: event participates in the linear chain (next Then waits).
  - isAsync=true: runs in parallel; use with care to avoid conflicting commands.

ActionPose registration and matching
- Register poses per-module before use (e.g., DriveModule.AddActionPose, SingleMotorModule.AddActionPose, SingleActuatorModule.AddActionPose).
- Matching is flexible with wildcards:
  - group matches or Group.Any
  - location/locationIndex/position match exact int or -1 as wildcard
  - action matches or Action.Any
- Ensure indices and group values align with your module’s defined poses (e.g., Approach vs Align). A typo like locationIndex 22 vs 12 will not match.

Motion groups semantics (Drive)
- Start: initial placement pose(s).
- Travel: uses a larger tolerance (frameNorm) and does not require a full stop; once within tolerance, lateral/forward are treated reached to allow smooth chaining to the next waypoint.
- Align/Approach: tighter alignment to scoring/interaction geometry.
- Score/Pickup: final approach plus mechanism actions.
- LookAt: if a pose provides a LookAt point, rotation is solved from the camera reference; when lateral and forward are reached, rotationReached is forced to avoid deadlock.

Settle and abandon
- Controllers mark each axis reached when within floatTolerance; settleCyclesRequired (default 3) consecutive ticks are required.
- When all axes remain reached for the window, AbandonTarget() is called:
  - Clears target, zeros commanded outputs, resets published target keys.
- Travel+LookAt: once position tolerance is reached, settle is forced (position axes reached), rotation may continue briefly depending on the lookAt target.

Swerve specifics
- Rotation PID: continuous input [-π, π] for shortest-path turning; lookAt rotations get a 1.5x boost currently for responsiveness.
- Module steering: SwerveModuleState.optimize chooses the shortest steering path, flipping drive if >90°.
- Pose estimation:
  - Initialized with Rotation2d.fromRadians(getGyroRadians()) and current wheel positions.
  - SetCurrentPose(new Pose3d) resets estimator and marks positionInitialized.
  - On real hardware, Positioner.GetPoseEstimate() is fused each tick.

Gyro/units conventions
- AHRSGyro.getAngle() returns CCW-positive degrees (negated NavX yaw), unbounded.
- Internals use radians for Rotation2d and PID math.
- Positioner.SetRobotOrientation receives degrees (convert via radiansToDegrees when needed).
- Simulation:
  - Gyro: appendSimValueDeg expects a degrees delta per tick (CCW+).
  - Encoder (REVEncoder): appendSimValueRad/Rot update rotations; velocity is derived as RPM assuming 20 ms loop.

SingleMotorModule (position and velocity)
- Position mode:
  - target.Measurement is rotations along the axis (continuous).
  - Encoder.getRawValue returns rotations (multiplier applied).
  - Module clamps motor output to [-1, 1].
- Velocity mode:
  - target.Measurement is RPM; enc.getVelocity (RPM) is compared directly.
  - For sim without hardware encoder, velocity is derived from rotation deltas as RPM.
  - sustainedDriveSpeed holds the final motor command for a few settle cycles to maintain speed, then clears.

SingleActuatorModule
- Relay-based actuator with an optional hold time (default ~200 ms) for forward/reverse pulses.
- Resets to Off each ProcessState tick unless a new target is active (safety by default-off).

Debugging (NetworkTables highlights)
- Drive module:
  - targetActionPose: current pose label or "none"
  - forward/lateral/rotationReached: booleans per axis
  - settleCount: current settle window count
  - currentPose: Pose3d publisher (AdvantageScope)
  - requestedModuleStates/actualModuleStates: SwerveModuleState arrays
  - rotationPidSetpoints/lateralPidSetpoints/forwardPidSetpoints: PID gains
  - lookTargetAng/lookTargetPos (when using LookAt)
- Motor modules:
  - currentDriveSpeed, rotationCount (rotations), velocity (RPM), delta metrics
- Positioner health:
  - positionerHealthy, positionHealthReason

Common recipes
- Pure wait:
  - AutoEventTime("Wait 500ms", false, 500, EventType.None, ac)
- Toggle field-oriented:
  - AutoEventTime("Enable FO", false, 60, EventType.Boolean, ac).SetBoolEvent(true, drive::SetFieldOriented)
- Mechanism pulse:
  - Open/close latches via SingleActuatorModule.ApplyValue/ApplyInverse using timed Boolean events
- Waypoint travel then align+score:
  - Travel waypoints (Group.Travel) → Align/Approach → Score + mechanism timed events; use AwaitTarget for each motion stage.

Pitfalls checklist
- Units: radians inside control; degrees only where APIs require (Positioner, gyro display).
- Sign: gyro is CCW-positive once in the wrapper; do not invert again elsewhere.
- Matching: wrong group or locationIndex values will silently miss; validate with drive.GetActionPose(...) at sequence init if needed.
- Timed events: for waits, use EventType.None instead of null consumers.
- Velocity mode: keep RPM consistent across real and sim; avoid mixing arbitrary scale factors into sim integration.
