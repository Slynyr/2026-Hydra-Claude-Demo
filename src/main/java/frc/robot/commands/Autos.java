package frc.robot.commands;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.GameCommandsConstants;
import frc.robot.Constants.kAutoAlign;
import frc.robot.RobotContainer;
import frc.robot.util.AutoPath;

import java.util.ArrayList;
import java.util.Objects;

import static edu.wpi.first.units.Units.*;

public class Autos {
    public static final EventTrigger autoPoseUpdate = new EventTrigger("Vision_Trigger");
	public static final Pose2d LEFT_BUMP_STARTING_POSE = new Pose2d(3.496, 5.585, Rotation2d.fromDegrees(45));
	public static final Pose2d RIGHT_BUMP_STARTING_POSE = new Pose2d(3.560,2.461, Rotation2d.fromDegrees(-45));

	public static ArrayList<AutoPath> getAutoPaths(RobotContainer robot) {
		ArrayList<AutoPath> autoPaths = new ArrayList<>();

		// LEFT SIDE AUTOS:

		autoPaths.add(
			new AutoPath(
				"LEFT-Bump-Intake-FarClose-Score",
                
				LEFT_BUMP_STARTING_POSE,

                // CROSS LEFT BUMP FROM ALLIANCE ZONE TO NEUTRAL ZONE 
                Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Alliance-Neutral"))
                    .beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
                    .andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),
				
				 
                // FOLLOW INTAKE PATH, FROM LEFT OF FIELD TOWARDS CENTER OF FIELD (ENDING VELOCITY OF 1.5 m/s) 
                Objects.requireNonNull(AutoPath.followPath("LEFT-INTAKE-FarClose"))
                    .alongWith(GameCommands.startIntake(robot)),


                // GO FROM ENDING OF INTAKE POSITION BACK TO BUMP POSITION 
                Objects.requireNonNull(AutoPath.followPath("LEFT-INTAKE-END-FarClose-To-BUMP")),
                

                // Go from NEUTRAL zone to ALLIANCE zone over LEFT BUMP
                Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Neutral-Alliance"))
                    .beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
                    .andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

                // LAUNCH FOR THE REMAINING DURATION OF AUTO
                GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
			)
		);

        autoPaths.add(
            new AutoPath(
		"LEFT-Bump-Intake-FarClose-Score-Depot-Score",
                
            	LEFT_BUMP_STARTING_POSE,

                // CROSS LEFT BUMP FROM ALLIANCE ZONE TO NEUTRAL ZONE
                Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Alliance-Neutral"))
                    .beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
                    .andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),


                // FOLLOW INTAKE PATH, FROM LEFT OF FIELD TOWARDS CENTER OF FIELD (ENDING VELOCITY OF 1.5 m/s)
                Objects.requireNonNull(AutoPath.followPath("LEFT-INTAKE-FarClose"))
                    .alongWith(GameCommands.startIntake(robot)),


                // GO FROM ENDING OF INTAKE POSITION BACK TO BUMP POSITION
                Objects.requireNonNull(AutoPath.followPath("LEFT-INTAKE-END-FarClose-To-BUMP")),


                // Go from NEUTRAL zone to ALLIANCE zone over LEFT BUMP
                Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Neutral-Alliance"))
                    .beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
                    .andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

                // LAUNCH FOR THE REMAINING DURATION OF AUTO
                Commands.deadline(
                    Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
                ),

                Objects.requireNonNull(AutoPath.followPath("LEFT-SCORE-to-DEPOT"))
                    .alongWith(GameCommands.startIntake(robot)),

                DriveCommands.alignToPoint(
						robot.sys_drive,
						() -> new  Pose2d(1.581, 5.711, Rotation2d.fromDegrees(170.194)),
						() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
						() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                        Centimeters.of(10),
                        Degrees.of(10),
                        MetersPerSecond.of(1)
				),
				GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
            )
        );

		autoPaths.add(
				new AutoPath(
					"LEFT-Bump-Intake-FarClose-Score-Bump",

					LEFT_BUMP_STARTING_POSE,

					// CROSS LEFT BUMP FROM ALLIANCE ZONE TO NEUTRAL ZONE
					Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Alliance-Neutral"))
							.beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
							.andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),


					// FOLLOW INTAKE PATH, FROM LEFT OF FIELD TOWARDS CENTER OF FIELD (ENDING VELOCITY OF 1.5 m/s)
					Objects.requireNonNull(AutoPath.followPath("LEFT-INTAKE-FarClose"))
							.alongWith(GameCommands.startIntake(robot)),


					// GO FROM ENDING OF INTAKE POSITION BACK TO BUMP POSITION
					Objects.requireNonNull(AutoPath.followPath("LEFT-INTAKE-END-FarClose-To-BUMP")),


					// Go from NEUTRAL zone to ALLIANCE zone over LEFT BUMP
					Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Neutral-Alliance"))
							.beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
							.andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

					// LAUNCH FOR AUTO_LAUNCH_WAIT_TIME amount of time before moving on
					Commands.deadline(
							Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
							GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
					),

					// ALIGN BACK TO BUMP TRAVERSE STARTING POSE
					DriveCommands.alignToPoint(
							robot.sys_drive,
							() -> LEFT_BUMP_STARTING_POSE,
							() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
							() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
					),

					//	GO BACK OVER BUMP
					Objects.requireNonNull(AutoPath.followPath("LEFT-BUMP-Alliance-Neutral"))
							.beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
							.andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false)))
							.andThen(GameCommands.startIntake(robot))
				)
		);

		// RIGHT SIDE AUTOS:
        autoPaths.add(
			new AutoPath(
				"RIGHT-Bump-Intake-FarClose-Score",

				new Pose2d(3.560,2.461, Rotation2d.fromDegrees(-45)),
				
                // cross RIGHT BUMP from ALLIANCE zone to NEUTRAL zone 
                Objects.requireNonNull(AutoPath.followPath("RIGHT-BUMP-Alliance-Neutral"))
                    .beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
                    .andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

                // follow INTAKE PATH, from RIGHT of field TOWARDS CENTER of field (ENDING VELOCITY OF 1.5 m/s)
				Objects.requireNonNull(AutoPath.followPath("RIGHT-INTAKE-FarClose"))
                    .alongWith(GameCommands.startIntake(robot)),

                // GO FROM ENDING OF INTAKE POSITION BACK TO BUMP POSITION 
                Objects.requireNonNull(AutoPath.followPath("RIGHT-INTAKE-END-FarClose-To-BUMP")),

				// Go from NEUTRAL zone to ALLIANCE zone over RIGHT BUMP
                Objects.requireNonNull(AutoPath.followPath("RIGHT-BUMP-Neutral-Alliance"))
                    .beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
                    .andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

                // LAUNCHES until auto ends
                GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)

			)
		);

		autoPaths.add(
				new AutoPath(
						"RIGHT-Bump-Intake-FarClose-Score-Bump",

						RIGHT_BUMP_STARTING_POSE,

						// cross RIGHT BUMP from ALLIANCE zone to NEUTRAL zone
						Objects.requireNonNull(AutoPath.followPath("RIGHT-BUMP-Alliance-Neutral"))
								.beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
								.andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

						// follow INTAKE PATH, from RIGHT of field TOWARDS CENTER of field (ENDING VELOCITY OF 1.5 m/s)
						Objects.requireNonNull(AutoPath.followPath("RIGHT-INTAKE-FarClose"))
								.alongWith(GameCommands.startIntake(robot)),

						// GO FROM ENDING OF INTAKE POSITION BACK TO BUMP POSITION
						Objects.requireNonNull(AutoPath.followPath("RIGHT-INTAKE-END-FarClose-To-BUMP")),

						// Go from NEUTRAL zone to ALLIANCE zone over RIGHT BUMP
						Objects.requireNonNull(AutoPath.followPath("RIGHT-BUMP-Neutral-Alliance"))
								.beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
								.andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

						// LAUNCHES for AUTO_LAUNCH_WAIT_TIME amount of time
						Commands.deadline(
								Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME),
								GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
						),

						// ALIGN BACK TO BUMP TRAVERSE STARTING POSE
						DriveCommands.alignToPoint(
								robot.sys_drive,
								() -> RIGHT_BUMP_STARTING_POSE,
								() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
								() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
						),

						//	GO BACK OVER BUMP
						Objects.requireNonNull(AutoPath.followPath("RIGHT-BUMP-Alliance-Neutral"))
								.beforeStarting(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(true)))
								.andThen(Commands.runOnce(() -> robot.sys_vision.setForceFusedIMU(false))),

						// follow INTAKE PATH, from RIGHT of field TOWARDS CENTER of field (ENDING VELOCITY OF 1.5 m/s)
						Objects.requireNonNull(AutoPath.followPath("RIGHT-INTAKE-FarClose"))
								.alongWith(GameCommands.startIntake(robot))


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

                GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
            )
        );

        autoPaths.add(
            new AutoPath(
                "Depot-Shoot",
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

                GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot))
        );

        autoPaths.add(
            new AutoPath(
                "Outpost-Shoot",
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
                    GameCommands.autoLaunch(() -> DriveCommands.distToHub(robot.sys_drive), () -> 0, () -> 0, robot)
                ),
                
                GameCommands.stopLaunching(robot)

                // GameCommands.autoClimb(drive, elevator, ClimbingPositions.RIGHT_PREP::getPose, ClimbingPositions.RIGHT::getPose)
            )
        );

		return autoPaths;
	}
	
}


// FAR-CLOSE Autos on hold
// LEFT:
// autoPaths.add(
		// 	new AutoPath(
		// 		"LeftBump-Intake-CloseFar-Score-LeftClimb",

		// 		// Angled Starting pose:
		// 		new Pose2d(3.565,5.400, new Rotation2d(Degrees.of(38.572))),

		// 		// Starting Pose: 
		// 		// new Pose2d(3.565,5.400, Rotation2d.k180deg),

		// 		// Alliance -> neutral zone 
		// 		DriveCommands.crossBump(
		// 			drive, 
		// 			vision,
		// 			drive::getRotation,
		// 			() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED), 
		// 			kBump.SETTLING_TIME
		// 		),

		// 		// confirm position
		// 		DriveCommands.alignToPoint(
		// 			drive,
		// 			() -> new Pose2d(6.200,5.400, new Rotation2d(Degrees.of(38.572))), 
		// 			() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
		// 			() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
		// 		),

        //         Objects.requireNonNull(AutoPath.followPath("Left-Bump-Intake-CloseFar"))
        //             .alongWith(GameCommands.startIntake(intake, hopper)),

		// 		// Align back to bump known position
		// 		DriveCommands.alignToPoint(
		// 			drive, 
		// 			() -> new Pose2d(6.200,(LinesHorizontal.leftBumpEnd + LinesHorizontal.leftBumpStart) / 2, Rotation2d.k180deg), 
		// 			() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
		// 			() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
		// 		),

        //         // TODO: DETERMINE IF INTAKE ROLLERS NEED TO BE STOPPED
        //         // intake.stopRoller(),

		// 		// neutral zone -> alliance zone
		// 		DriveCommands.crossBump(
		// 			drive, 
		// 			vision,
		// 			drive::getRotation,
		// 			() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
		// 			kBump.SETTLING_TIME
		// 		),

        //         Commands.deadline(
        //             Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME), 
        //             GameCommands.autoLaunch(() -> DriveCommands.distToHub(drive), drive, launcher, feeder, serializer, intake)
        //         ),
                
        //         GameCommands.stopLaunching(launcher, feeder, serializer, intake)

        //         // GameCommands.autoClimb(drive, elevator, ClimbingPositions.LEFT_PREP::getPose, ClimbingPositions.LEFT::getPose)
		// 	)
		// );


// RIGHT:
		// autoPaths.add(
		// 	new AutoPath(
		// 		"RightBump-Intake-CloseFar-Score-RightClimb",

		// 		// Starting Pose: 
		// 		// new Pose2d(3.565,2.750, Rotation2d.kZero),

		// 		// Angled Starting Pose
		// 		new Pose2d(3.565,2.750, new Rotation2d(Degrees.of(-41.689))),
				
		// 		// Alliance -> neutral zone
		// 		DriveCommands.crossBump(
		// 			drive, 
		// 			vision,
		// 			drive::getRotation,
		// 			() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED), 
		// 			kBump.SETTLING_TIME
		// 		),

		// 		// confirm position
		// 		DriveCommands.alignToPoint(
		// 			drive, 

		// 			() -> new Pose2d(6.265,2.750, new Rotation2d(Degrees.of(-41.689))), 
		// 			() -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
		// 			() -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION
		// 		),
		// 		// Follow path from center of neutral zone to right of field
		// 		Objects.requireNonNull(AutoPath.followPath("Right-Bump-Intake-CloseFar"))
        //         .alongWith(GameCommands.startIntake(intake, hopper)),

		// 		// Align back to bump known position
		// 		DriveCommands.alignToPoint(
		// 			drive, 
		// 			() -> new Pose2d(6.200,(LinesHorizontal.rightBumpStart + LinesHorizontal.rightBumpEnd) / 2, Rotation2d.k180deg), 
		// 			() -> MetersPerSecond.of(2.0), 
		// 			() -> MetersPerSecondPerSecond.of(8.0)
		// 		),

        //         // TODO: DETERMINE IF NEEDED TO STOP INTAKING
        //         // intake.stopRoller(),

		// 		// neutral zone -> alliance zone
		// 		DriveCommands.crossBump(
		// 			drive, vision,
		// 			drive::getRotation,
		// 			() -> DriveCommands.getBumpSpeed(kBump.BUMP_TRAVERSAL_SPEED.times(-1)), 
		// 			kBump.SETTLING_TIME
		// 		),

        //         Commands.deadline(
        //             Commands.waitTime(GameCommandsConstants.AUTO_LAUNCH_WAIT_TIME), 
        //             GameCommands.autoLaunch(() -> DriveCommands.distToHub(drive), drive, launcher, feeder, serializer, intake)
        //         ),
                
        //         GameCommands.stopLaunching(launcher, feeder, serializer, intake)

        //         // GameCommands.autoClimb(drive, elevator, ClimbingPositions.RIGHT_PREP::getPose, ClimbingPositions.RIGHT::getPose)
		// 	)
		// );
