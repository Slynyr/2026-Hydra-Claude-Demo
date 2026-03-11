// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.GameCommandsConstants;
import frc.robot.Constants.kAutoAlign;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.subsystems.serializer.SerializerConstants;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Milliseconds;

public class GameCommands {

    public static Command autoLaunch(Supplier<Distance> distanceSupplier, RobotContainer robot) {
        return Commands.parallel(
                DriveCommands.alignToHeading(
                        robot.sys_drive,
                        () -> DriveCommands.getRotationToHub(robot.sys_drive)
                ),
                Commands.sequence(
                        Commands.parallel(
                                Commands.waitUntil(DriveCommands::isAligned),
                                robot.sys_launcher.launchFuel(distanceSupplier, robot.sys_feeder)

                        ),
                        Commands.waitUntil(robot.sys_launcher::isLauncherAtSpeed),

                        robot.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                        Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                        agitateIntake(robot.sys_intake)
                )
        );
    }

    public static Command manualLaunch(Supplier<Distance> distance, RobotContainer robot) {
        return Commands.sequence(
                robot.sys_launcher.launchFuel(distance, robot.sys_feeder),

                Commands.waitUntil(robot.sys_launcher::isLauncherAtSpeed),

                robot.sys_elevator.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                agitateIntake(robot.sys_intake)
        );
    }

    /**
     * Drive aligns to face target manually
     */
    public static Command manualPass(RobotContainer robot) {
        return Commands.sequence(

                Commands.parallel(
                        robot.sys_launcher.runVelocity(() -> GameCommandsConstants.PASSING_RPS),
                        robot.sys_launcher.setHoodExtension(() -> GameCommandsConstants.PASSING_HOOD_ANGLE)

                ),

                Commands.waitUntil(robot.sys_launcher::isLauncherAtSpeed),

                robot.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                agitateIntake(robot.sys_intake)

        );
    }

    public static Command startIntake(RobotContainer robot) {
        return Commands.sequence(
                robot.sys_hopper.extend(),
                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_INTAKE_EXTENSION),
                Commands.parallel(
                        robot.sys_intake.extend(),
                        robot.sys_intake.setRollerVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE)
                )
        );
    }

    public static Command retract(RobotContainer robot) {
        return Commands.parallel(
                robot.sys_intake.retract(),
                robot.sys_hopper.retract(),
                robot.sys_intake.stopRoller()
        );
    }

    public static Command agitateIntake(Intake intake) {
        return Commands.parallel(
                intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
                Commands.repeatingSequence(
                        intake.setSetpoint(() -> Extension.RETRACT_POINT),
                        Commands.waitTime(Milliseconds.of(1000)),
                        intake.setSetpoint(() -> Extension.EXTEND_POINT),
                        Commands.waitTime(Milliseconds.of(1000))
                )
        );
    }

    public static Command autoClimb(RobotContainer robot, Supplier<Pose2d> prepPose, Supplier<Pose2d> climbPose) {
        return Commands.sequence(
                Commands.parallel(
                        DriveCommands.alignToPoint(
                                robot.sys_drive,
                                prepPose,
                                () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                                () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                                kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                                kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                                kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
                        ),
                        robot.sys_elevator.setSetpointAndWait(ElevatorConstants.kSetpoints.ELEVATOR_UP, 0)
                ),

                DriveCommands.alignToPoint(
                        robot.sys_drive,
                        climbPose,
                        () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
                        () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
                ),

                robot.sys_elevator.setSetpointAndWait(ElevatorConstants.kSetpoints.ELEVATOR_DOWN, 0)
        );
    }

    /**
     * Stops launcher, feeder and calls {@link GameCommands#stopSerializing(RobotContainer)}
     */
    public static Command stopLaunching(RobotContainer robot) {
        return Commands.parallel(
                robot.sys_launcher.stopLauncher(),
                robot.sys_feeder.stop(),
                stopSerializing(robot)
        );
    }

    public static Command stopSerializing(RobotContainer robot) {
        return Commands.parallel(
                robot.sys_serializer.stop(),
                robot.sys_intake.stop(),
                robot.sys_intake.stopRoller()
        );
    }
}
