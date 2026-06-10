# Team 5409 Robot Code — Standards

## Repository Layout
- `src/main/java/frc/robot/` — all Java source
  - `Robot.java` / `RobotContainer.java` — entry points; RobotContainer wires subsystems and commands
  - `Constants.java` — global tunable values (field-wide, robot-wide)
  - `commands/` — `Autos.java` (auto routines), `DriveCommands.java`, `GameCommands.java`
  - `subsystems/` — one folder per mechanism: `drive`, `elevator`, `feeder`, `fuelgauge`, `hopper`, `intake`, `launcher`, `serializer`, `vision`
    - Each subsystem folder contains: `<Name>.java` (subsystem), `<Name>Constants.java`, `<Name>IO.java` (interface), `<Name>IOTalonFX.java` (real impl), `<Name>IOSim.java` (sim impl)
  - `generated/TunerConstants.java` — CTRE Tuner X auto-generated swerve constants (do not hand-edit)
  - `util/` — shared helpers (`FieldConstants`, `MathUtils`, `LogHelper`, `LimelightHelpers`, etc.)
- `src/main/deploy/` — files deployed to the RoboRIO: AprilTag layouts (`apriltags/`), PathPlanner paths (`pathplanner/paths/`)
- `AdvantageScope/` — robot log layout configs for AdvantageScope

## Framework
- WPILib command-based (Java). Subsystems own hardware; commands declare requirements.
- All tunable values live in `Constants`. No magic numbers in subsystem/command logic.
- Units: store and compute in meters and radians internally; convert only at I/O boundaries.

## Review posture
- Be direct. Identify the issue, explain why it matters in one or two lines, and give the fix.
- Flag anything that could cause a brownout, watchdog overrun, or unsafe actuator behavior as high priority.
- Call out blocking calls (Thread.sleep, busy-waits) anywhere in the main robot loop.

## Conventions
- Subsystem files: PascalCase ending in `Subsystem`. Commands: PascalCase ending in `Command`.
- Every motor controller config sets a current limit.