# Team 5409 Robot Code — Standards

## Repository Layout
- `src/main/java/frc/robot/` — all Java source
  - `Robot.java` / `RobotContainer.java` — entry points; RobotContainer wires subsystems and commands
  - `Constants.java` — global tunable values; `Constants.DeviceID` holds all CAN IDs and Limelight name
  - `commands/` — `Autos.java` (auto routines), `DriveCommands.java`, `GameCommands.java`
  - `subsystems/` — one folder per mechanism: `drive`, `elevator`, `feeder`, `fuelgauge`, `hopper`, `intake`, `launcher`, `serializer`, `vision`
    - Each subsystem folder: `<Name>.java`, `<Name>Constants.java`, `<Name>IO.java` (interface), `<Name>IOTalonFX.java` (real), `<Name>IOSim.java` (sim)
  - `generated/TunerConstants.java` — CTRE Tuner X auto-generated swerve constants (do not hand-edit)
  - `util/` — `FieldConstants`, `MathUtils`, `LogHelper`, `LimelightHelpers`, `PhoenixUtil`, `AutoPath`, `Checkmate`
- `src/main/deploy/` — AprilTag layouts (`apriltags/`), PathPlanner paths (`pathplanner/paths/`)

## Framework
- WPILib command-based (Java). Subsystems own hardware; commands declare requirements.
- All tunable values live in `Constants`. No magic numbers in subsystem/command logic.
- Units: meters and radians internally; convert only at I/O boundaries.
- IO abstraction via AdvantageKit `@AutoLog` — every subsystem logs inputs through its IO interface.
- Motor configs applied via `PhoenixUtil.tryUntilOk()` retry wrapper for robust CAN setup.

## Review Posture
- Be direct and concise: one line identifying the issue, one line why it matters, then the fix.
- Every finding must include a concrete fix, not just a description.
- Flag **[HIGH]** for anything that could cause a brownout, watchdog overrun, unsafe actuator behavior, or data loss.
- Flag **[MEDIUM]** for logic errors, missing requirements, incorrect unit conversions.
- Flag **[LOW]** for style, dead code, missing constants extraction.

## Safety Rules — Flag Any Violation
- **No blocking calls** (`Thread.sleep`, busy-waits, long synchronous I/O) anywhere in the main robot loop or `periodic()`. Known existing violation: `Vision.java` uses `Thread.sleep(20)` in `periodic()` — any new occurrences are regressions.
- **Every TalonFX config must set both supply and stator current limits** via `CurrentLimitsConfigs`. Missing limits = brownout risk.
- **Servo position must be bounded** by software limits before commanding. Hood servos use pulse-width ranges (1000–2000 µs) — commands outside `LauncherConstants.Hood.MAX_EXTENSION` are unsafe.
- **Crash detection hooks exist** in `Intake.java` and `Hopper.java` but are currently disabled. Do not silently remove them; note if new code bypasses these.

## Command Conventions
- Commands composed via `Commands.sequence/parallel/deadline/defer/waitUntil` — never block inside `execute()`.
- Subsystem requirements must be declared either via passing the subsystem to the command factory or via `addRequirements()`. Missing requirements = silent resource contention.
- `Commands.defer()` is used to evaluate targets at execution time (e.g., launcher distance calculation) — flag any eager evaluation that should be deferred.
- Triggers in `RobotContainer` drive persistent behaviors (e.g., launcher idle at 40 RPS). New persistent behaviors belong there, not in `periodic()`.

## Vision
- Pose updates accepted only if `tagCount >= FIDUCIAL_TRUST_THRESHOLD` and `avgTagDist <= MAX_TAG_DISTANCE_METERS`.
- Std dev scales quadratically with distance: `xy = base + per_meter * dist²`. Flag any hardcoded std devs.
- IMU mode changes on enable/disable transitions — do not call `setIMUMode` from subsystem `periodic()`.

## Conventions
- Subsystem files: PascalCase ending in `Subsystem`. Commands: PascalCase ending in `Command`.
- Every motor controller config sets a current limit.
- Dead/deprecated code should be deleted, not commented out. (`prepClimberPositionCommand()` is a known leftover.)
