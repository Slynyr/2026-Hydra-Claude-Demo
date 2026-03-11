package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.GameCommandsConstants;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kBump;
import frc.robot.RobotContainer;
import frc.robot.util.AutoPath;
import frc.robot.util.FieldConstants.LinesHorizontal;

import java.util.ArrayList;
import java.util.Objects;

import static edu.wpi.first.units.Units.*;

public class Autos {

	public static ArrayList<AutoPath> getAutoPaths(RobotContainer robot) {
		ArrayList<AutoPath> autoPaths = new ArrayList<>();

		// LEFT SIDE AUTOS:
		autoPaths.add(
			new AutoPath(
				"LeftBump-Intake-CloseFar-Score-LeftClimb",

				// Angled Starting pose:
				new Pose2d(3.565,5.400, new Rotation2d(Degrees.of(38.572))),

				// Starting Pose:
				// new Pose2d(3.565,5.400, Rotation2d.k180deg),

				// Alliance -> neutral zone
				DriveCommands.crossBump(
					robot.sys_drive,
					robot.sys_vision,
					robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED),
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
                    robot.sys_drive,
					() -> new Pose2d(6.200,5.400, new Rotation2d(Degrees.of(38.572))),
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

                Objects.requireNonNull(AutoPath.followPath("Left-Bump-Intake-CloseFar"))
                    .alongWith(GameCommands.startIntake(robot)),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					robot.sys_drive,
					() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg),
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

                // TODO: DETERMINE IF INTAKE ROLLERS NEED TO BE STOPPED
                // intake.stopRoller(),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					robot.sys_drive,
					robot.sys_vision,
					robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)),
					kBump.SETTLING_TIME
				),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.LEFT_PREP::getPose, ClimbingPositions.LEFT::getPose)
			)
		);

		autoPaths.add(
			new AutoPath(
				"LeftBump-Intake-FarClose-Score-LeftClimb",
				// Angled Start:
				new Pose2d(3.565,5.801, new Rotation2d(Degrees.of(-43.361))),
				// Starting Pose:
				// new Pose2d(3.565,5.801, Rotation2d.k180deg),

				// Alliance -> neutral zone
				DriveCommands.crossBump(
					robot.sys_drive,
					robot.sys_vision,
					robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED),
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					robot.sys_drive,
					() -> new Pose2d(6.187,5.969, new Rotation2d(Degrees.of(-43.361))),
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

				// Follow path from center of neutral zone to left of field
                Objects.requireNonNull(AutoPath.followPath("Left-Bump-Intake-FarClose"))
                    .alongWith(GameCommands.startIntake(robot)),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					robot.sys_drive,
					() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg),
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),

                // TODO: DETERMINE IF WE NEED TO STOP INTAKE ROLLERS
                // intake.stopRoller(),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
					robot.sys_drive,
					robot.sys_vision,
					robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)),
					kBump.SETTLING_TIME
				),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.LEFT_PREP::getPose, ClimbingPositions.LEFT::getPose)
			)
		);

		// RIGHT SIDE AUTOS:
		autoPaths.add(
			new AutoPath(
				"RightBump-Intake-CloseFar-Score-RightClimb",

				// Starting Pose:
				// new Pose2d(3.565,2.750, Rotation2d.kZero),

				// Angled Starting Pose
				new Pose2d(3.565,2.750, new Rotation2d(Degrees.of(-41.689))),

				// Alliance -> neutral zone
				DriveCommands.crossBump(
                    robot.sys_drive,
                    robot.sys_vision,
                    robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED),
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					robot.sys_drive,

					() -> new Pose2d(6.265,2.750, new Rotation2d(Degrees.of(-41.689))),
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),
				// Follow path from center of neutral zone to right of field
				Objects.requireNonNull(AutoPath.followPath("Right-Bump-Intake-CloseFar"))
                .alongWith(GameCommands.startIntake(robot)),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					robot.sys_drive,
					() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg),
					() -> MetersPerSecond.of(2.0),
					() -> MetersPerSecondPerSecond.of(8.0)
				),

                // TODO: DETERMINE IF NEEDED TO STOP INTAKING
                // intake.stopRoller(),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
                    robot.sys_drive,
                    robot.sys_vision,
                    robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)),
					kBump.SETTLING_TIME
				),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.RIGHT_PREP::getPose, ClimbingPositions.RIGHT::getPose)
			)
		);

		// Right Side
		// Bump
		// Intake from the edge of the field to the center
		// Go back and score
		// Climb
		autoPaths.add(
			new AutoPath(
				"RightBump-Intake-FarClose-Score-RightClimb",

				// Starting Pose:
				// new Pose2d(3.565,2.750, Rotation2d.k180deg),

				// Angled Starting Pose
				new Pose2d(3.565,2.282, new Rotation2d(Degrees.of(53.181))),

				// Alliance -> neutral zone
				DriveCommands.crossBump(
                    robot.sys_drive,
                    robot.sys_vision,
                    robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED),
					kBump.SETTLING_TIME
				),

				// confirm position
				DriveCommands.alignToPoint(
					robot.sys_drive,
					() -> new Pose2d(6.200,2.282, new Rotation2d(Degrees.of(53.181))),
					() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
					() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
				),
				// Follow path from right of field to center of neutral zone
				Objects.requireNonNull(AutoPath.followPath("Right-Bump-Intake-FarClose"))
                .alongWith(GameCommands.startIntake(robot)),

				// Align back to bump known position
				DriveCommands.alignToPoint(
					robot.sys_drive,
					() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg),
					() -> MetersPerSecond.of(2.0),
					() -> MetersPerSecondPerSecond.of(8.0)
				),

                // TODO: DETERMINE IF THERE IS A NEED TO STOP INTAKE ROLLERS
                // intake.stopRoller(),

				// neutral zone -> alliance zone
				DriveCommands.crossBump(
                    robot.sys_drive,
                    robot.sys_vision,
                    robot.sys_drive::getRotation,
					() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)),
					kBump.SETTLING_TIME
				),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.RIGHT_PREP::getPose, ClimbingPositions.RIGHT::getPose)
			)
		);

        if (Constants.IS_TUNING)
            autoPaths.add(
                new AutoPath(
                    "Test-Path",
                    new Pose2d(2,7,Rotation2d.k180deg),
                    DriveCommands.alignToPoint(
                        robot.sys_drive,
                        () -> new Pose2d(2,7,Rotation2d.kZero),
                        () -> MetersPerSecond.of(1),
                        () -> MetersPerSecondPerSecond.of(2)
                    ),
                    AutoPath.followPath("TestPath")
                )
            );


        autoPaths.add(
            new AutoPath(
                "Leave-Shoot",
                new Pose2d(3.565,4.011,Rotation2d.k180deg),
                DriveCommands.alignToPoint(
                    robot.sys_drive,
                    () -> new Pose2d(3.127,4.011,Rotation2d.k180deg),
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)
            )
        );

        autoPaths.add(
            new AutoPath(
                "Depot-Shoot-LeftClimb",
                new Pose2d(3.565,5.958,Rotation2d.k180deg),

                // Go from starting point to depot
                Objects.requireNonNull(AutoPath.followPath("Start-Depot"))
                .alongWith(GameCommands.startIntake(robot)),

                // Align to scoring point
                DriveCommands.alignToPoint(
                    robot.sys_drive,
                    () -> new Pose2d(1.390, 4.887, Rotation2d.k180deg),
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
                ),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.LEFT_PREP::getPose, ClimbingPositions.LEFT::getPose)

            )
        );

        autoPaths.add(
            new AutoPath(
                "Outpost-Shoot-RightClimb",
                // Start at edge of bump
                new Pose2d(3.565,2.076,Rotation2d.k180deg),
                // Start at trench
                // new Pose2d(3.565,0.719,Rotation2d.k180deg),
                DriveCommands.alignToPoint(
                    robot.sys_drive,
                    () -> new Pose2d(0.473, 0.670, Rotation2d.k180deg),
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
                )
                .alongWith(GameCommands.startIntake(robot)),
                // Time to wait for outpost dump
                Commands.waitSeconds(2),

                DriveCommands.alignToPoint(
                    robot.sys_drive,
                    () -> new Pose2d(0.902, 0.670, Rotation2d.k180deg),
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
                ),

                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), robot)
                ),

                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.RIGHT_PREP::getPose, ClimbingPositions.RIGHT::getPose)
            )
        );

		return autoPaths;
	}

}
