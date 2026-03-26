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
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.subsystems.serializer.SerializerConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class GameCommands {

    public static Command autoLaunch(
            Supplier<Distance> distanceSupplier,
            DoubleSupplier joystickX,
            DoubleSupplier joystickY,
            RobotContainer robot
    ) {
        return Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                        robot.sys_drive,
                        joystickX,
                        joystickY,
                        () -> DriveCommands.getRotationToHub(robot.sys_drive)
                ),
                Commands.sequence(
                        Commands.runOnce(() -> Logger.recordOutput("GameCommands/StartingLaunchSequence", true)),
                        Commands.parallel(
                                Commands.waitUntil(DriveCommands::isAligned),
                                robot.sys_launcher.launchFuel(distanceSupplier, robot.sys_feeder)

                        ),
                        Commands.waitUntil(robot.sys_launcher::isLauncherAtSpeed),

                        robot.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                        Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                        agitateThenRetract(robot)
                )
        );
    }

    public static Command manualLaunch(Supplier<Distance> distance, RobotContainer robot) {
        return Commands.sequence(
                robot.sys_launcher.launchFuel(distance, robot.sys_feeder),

                Commands.waitUntil(robot.sys_launcher::isLauncherAtSpeed),

                robot.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                agitateThenRetract(robot)
        );
    }

    /**
     * Drive aligns to face target manually
     */
    public static Command manualPass(
            BooleanSupplier isRightHalf,
            DoubleSupplier joystickX,
            DoubleSupplier joystickY,
            RobotContainer robot
    ){
        return Commands.parallel(
                DriveCommands.joystickDriveAtAngle(
                        robot.sys_drive,
                        joystickX,
                        joystickY,
                        () -> DriveCommands.getRotationToPassingPosition(robot.sys_drive, isRightHalf)
                ),
                Commands.sequence(
                        Commands.parallel(
                                robot.sys_launcher.runVelocity(() -> GameCommandsConstants.PASSING_RPS),
                                robot.sys_launcher.setHoodExtension(() -> GameCommandsConstants.PASSING_HOOD_ANGLE),
                                robot.sys_feeder.runVelocity(() -> GameCommandsConstants.PASSING_RPS)
                        ),

                        Commands.waitUntil(robot.sys_launcher::isLauncherAtSpeed),

                        robot.sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                        Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                        agitateThenRetract(robot)
                )

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

    public static Command agitateSystem(RobotContainer robot) {
        return Commands.parallel(
                robot.sys_intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
                Commands.repeatingSequence(
                        robot.sys_intake.setSetpoint(() -> Extension.RETRACT_POINT)
                                .alongWith(robot.sys_hopper.setSetpoint(() -> HopperConstants.RETRACT_POINT)),
                        Commands.waitTime(Milliseconds.of(150)),
                        robot.sys_intake.setSetpoint(() -> Extension.EXTEND_POINT)
                                .alongWith(robot.sys_hopper.setSetpoint(() -> HopperConstants.EXTEND_POINT)),
                        Commands.waitTime(Milliseconds.of(150))
                )
        );
    }

    public static Command agitateThenRetract(RobotContainer robot) {
        return Commands.sequence(
                agitateSystem(robot).withTimeout(Seconds.of(3.5)),
                Commands.parallel(
                        robot.sys_intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
                        robot.sys_intake.retract(),
                        robot.sys_hopper.retract()
                )
        );
    }


    // public static Command autoClimb(RobotContainer robot, Supplier<Pose2d> prepPose, Supplier<Pose2d> climbPose) {
    //     return Commands.sequence(
    //             Commands.parallel(
    //                     DriveCommands.alignToPoint(
    //                             robot.sys_drive,
    //                             prepPose,
    //                             () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
    //                             () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
    //                             kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
    //                             kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
    //                             kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
    //                     ),
    //                     robot.sys_elevator.setSetpointAndWait(ElevatorConstants.kSetpoints.ELEVATOR_UP, 0)
    //             ),

    //             DriveCommands.alignToPoint(
    //                     robot.sys_drive,
    //                     climbPose,
    //                     () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
    //                     () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
    //             ),

    //             robot.sys_elevator.setSetpointAndWait(ElevatorConstants.kSetpoints.ELEVATOR_DOWN, 0)
    //     );
    // }

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
                robot.sys_intake.stopRoller(),
                robot.sys_hopper.stopMotor()
        );
    }

    public static Command reverseRollers(RobotContainer robot) {
        return Commands.parallel(
                robot.sys_serializer.setVoltage(-SerializerConstants.SERIALIZING_VOLTAGE),
                robot.sys_feeder.setVoltage(-FeederConstants.FEEDER_REVERSE_VOLTAGE),
                robot.sys_launcher.runVelocity(() -> RotationsPerSecond.of(-20))
        );
    }
}
