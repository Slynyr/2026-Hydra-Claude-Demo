// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.serializer.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.Constants.*;

/** Add your docs here. */
public class GameCommands {

    public static Command autoLaunch(Supplier<Distance> distanceSupplier, Drive drive, Launcher launcher, Feeder feeder, Serializer serializer, Intake intake){
        return Commands.parallel(
            DriveCommands.alignToHeading(
                drive,
                () -> DriveCommands.getRotationToHub(drive)
            ),
            Commands.sequence(
                Commands.parallel(
                    Commands.waitUntil(DriveCommands::isAligned),
                    launcher.launchFuel(distanceSupplier, feeder)
                    
                ),
                Commands.waitUntil(launcher::isLauncherAtSpeed), 

                serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

                Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

                agitateIntake(intake)
            )
        );
    }

    public static Command manualLaunch(Supplier<Distance> distance, Launcher launcher, Feeder feeder, Serializer serializer, Intake intake ){
        return Commands.sequence(
            
            launcher.launchFuel(distance, feeder),

            Commands.waitUntil(launcher::isLauncherAtSpeed),
            
            serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

            agitateIntake(intake)
        );
    }


    /**
     * Drive aligns to face target manually
     * @param distanceSupplier
     * @param launcher
     * @param feeder
     * @param serializer
     * @param intake
     * @return
     */
    public static Command manualPass(Launcher launcher, Feeder feeder, Serializer serializer, Intake intake){
        return Commands.sequence(

            Commands.parallel(
                launcher.runVelocity(() -> GameCommandsConstants.PASSING_RPS),
                launcher.setHoodExtension(() -> GameCommandsConstants.PASSING_HOOD_ANGLE)
                
            ),
            
            Commands.waitUntil(launcher::isLauncherAtSpeed),

            serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE),

            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_AGITATE),

            agitateIntake(intake)

        );
    }

    public static Command startIntake(Intake intake, Hopper hopper){
        return Commands.sequence(
            hopper.fullExtend(),
            Commands.waitTime(GameCommandsConstants.WAIT_TIME_BEFORE_INTAKE_EXTENSION),
            Commands.parallel(
                intake.extend(),
                intake.setRollerVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE)
            )
        );
    }

    public static Command retract(Intake intake, Hopper hopper){
        return Commands.parallel(
          intake.retract(),
          hopper.fullRetract(),
          intake.stopRoller()  
        );
    }

    public static Command agitateIntake(Intake intake){
        return Commands.parallel(
            intake.setRollerVoltage(IntakeConstants.Roller.AGITATE_VOLTAGE),
            Commands.repeatingSequence(
                intake.move(() -> GameCommandsConstants.RETRACT_POINT),
                Commands.waitTime(Milliseconds.of(1000)),
                intake.move(() -> GameCommandsConstants.EXTEND_POINT),
                    Commands.waitTime(Milliseconds.of(1000))
            )
        );
    }

    public static Command autoClimb(Drive drive, Elevator elevator, Supplier<Pose2d> prepPose, Supplier<Pose2d> climbPose){
        return Commands.sequence(
            Commands.parallel(
                DriveCommands.alignToPoint(
                    drive,
                    prepPose,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY,
                    () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                    kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                    kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP
                ),
                elevator.elevatorGo(ElevatorConstants.kSetpoints.ELEVATOR_UP,0)
            ),

            DriveCommands.alignToPoint(
                drive,
                climbPose,
                () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB,
                () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
            ),

            elevator.elevatorGo(ElevatorConstants.kSetpoints.ELEVATOR_DOWN,0)
        );
    }

    /**
     * Stops launcher, feeder and calls {@link GameCommands#stopSerializing(Serializer, Intake)}
     * @param launcher
     * @param feeder
     * @param serializer
     * @param intake
     * @return
     */
    public static Command stopLaunching(Launcher launcher, Feeder feeder, Serializer serializer, Intake intake){
        return Commands.parallel(
            launcher.stopLauncher(),
            feeder.stopMotor(),
            stopSerializing(serializer, intake)
        );
    }

    public static Command stopSerializing(Serializer serializer, Intake intake){
        return Commands.parallel(
            serializer.stopMotor(),
            intake.stopMotor(),
            intake.stopRoller()
        );
    }
}
