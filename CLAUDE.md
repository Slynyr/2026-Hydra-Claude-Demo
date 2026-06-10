# Team 5409 Robot Code — Standards

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