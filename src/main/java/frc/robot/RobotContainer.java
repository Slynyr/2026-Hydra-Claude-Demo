// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import frc.robot.Constants.DeviceID;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimbingPositions;
import frc.robot.Constants.PassingPositions;
import frc.robot.Constants.kAutoAlign;
import frc.robot.Constants.kBump;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.subsystems.intake.IntakeConstants.Roller;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.serializer.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.FieldConstants.Hub;
import frc.robot.commands.*;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Inches;
import frc.robot.Constants.DeviceID;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Amps;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final Drive      sys_drive;
    protected final Vision     sys_vision;
    protected final Intake     sys_intake;
    protected final Serializer sys_serializer;
    protected final Feeder     sys_feeder;
    protected final Hopper     sys_hopper;

    public static SwerveDriveSimulation simConfig;
    private final Elevator sys_elevator;

    private PassingPositions selectedPassingPosition = PassingPositions.MIDDLE;
    private ClimbingPositions selectedClimbingPosition = ClimbingPositions.LEFT;
    private ClimbingPositions selectedClimibingPrepPosition = ClimbingPositions.LEFT_PREP;

    // Controllers
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);
    private final CommandXboxController tertiaryController = new CommandXboxController(2);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            // Real robot, instantiate hardware IO implementations
            case REAL -> {
                sys_hopper = new Hopper(
                        new HopperIOTalonFX(DeviceID.HOPPER_MOTOR_ID));
                sys_intake = new Intake(new IntakeIOTalonFX(DeviceID.INTAKE_ROLLER_MOTOR, DeviceID.INTAKE_EXTENSION_MOTOR));
                sys_serializer = new Serializer(
                        new SerializerIOTalonFX(DeviceID.SERIALIZER_MOTOR, DeviceID.FEEDER_MOTOR_BOTTOM));
                sys_feeder = new Feeder(new FeederIOTalonFX(DeviceID.FEEDER_MOTOR_TOP));
                sys_vision = new Vision(new VisionIOLimelight());
                sys_elevator = new Elevator(new ElevatorIOTalonFX(DeviceID.CLIMBER_MOTOR));


                sys_drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        sys_vision
                );
            }
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                sys_hopper = new Hopper(new HopperIOSim());
                sys_intake = new Intake(new IntakeIOSim());
                sys_serializer = new Serializer(new SerializerIOSim());
                sys_elevator = new Elevator(new ElevatorIOSim());
                sys_feeder = new Feeder(new FeederIOSim());

                final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig
                        .Default()
                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(DriveConstants.ROBOT_FULL_MASS)
                        .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
                        .withBumperSize(Meters.of(0.881), Meters.of(0.881))
                        .withSwerveModule(
                                COTS.ofMark4i(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getKrakenX60(1),
                                        DriveConstants.WHEEL_COF,
                                        1
                                )
                        );

                simConfig = new SwerveDriveSimulation(
                        driveConfig,
                        new Pose2d(3, 3, Rotation2d.kZero)
                );

                SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
                SimulatedArena.getInstance().addDriveTrainSimulation(simConfig);
                SimulatedArena.getInstance().resetFieldForAuto();

                sys_vision = new Vision(new VisionIOSim(simConfig));

                sys_drive = new Drive(
                        new GyroIOSim(simConfig.getGyroSimulation()),
                        new ModuleIOSim(simConfig.getModules()[0]),
                        new ModuleIOSim(simConfig.getModules()[1]),
                        new ModuleIOSim(simConfig.getModules()[2]),
                        new ModuleIOSim(simConfig.getModules()[3]),
                        sys_vision
                );
            }
            // Replayed robot, disable IO implementations
            default -> {
                sys_vision = new Vision(new VisionIO() {});
                sys_drive = new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        sys_vision);
                sys_hopper = new Hopper(new HopperIO() {});
                sys_intake = new Intake(new IntakeIO() {});
                sys_serializer = new Serializer(new SerializerIO() {});
                sys_elevator = new Elevator(new ElevatorIO() {});
                sys_feeder = new Feeder(new FeederIO() {});
            }
        }

        // Set up auto routines
        autoChooser = buildAutoChooser();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * builds the dashboard command chooser ({@link LoggedDashboardChooser}) for picking autonomous routines.
     *
     * @return the logged dashboard chooser
     */
    private LoggedDashboardChooser<Command> buildAutoChooser() {
        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>(
                "Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        chooser.addOption(
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(sys_drive));
        chooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(sys_drive));
        chooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        chooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        chooser.addOption(
                "Drive SysId (Dynamic Forward)",
                sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        chooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        return chooser;
    }

    /**
     * Updates sim positions of algae, coral and robot poses
     */
    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        sys_drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        sys_drive,
                        () -> -primaryController.getLeftY(),
                        () -> -primaryController.getLeftX(),
                        () -> -(primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
                )
        );

        // Switch to X pattern when X button is pressed
        primaryController.x()
                         .onTrue(Commands.runOnce(sys_drive::stopWithX, sys_drive));

        // Switch To Bump Speed Modifier
        primaryController.a()
                         .onTrue(Commands.runOnce(() -> DriveCommands.setSpeed(kBump.BUMP_SPEED_MODIFIER)))
                         .onFalse(Commands.runOnce(() -> DriveCommands.setSpeed(1.0)));
    
        tertiaryController.y().onTrue(Commands.runOnce(() -> sys_elevator.goTillSpike(-3)));
        tertiaryController.povUp().onTrue(Commands.runOnce(() -> sys_elevator.startManualMove(0.5)));
        tertiaryController.povDown().onTrue(Commands.runOnce(() -> sys_elevator.startManualMove(-0.5)));

        primaryController.rightBumper()
                         .whileTrue(
                              DriveCommands.alignToHeading(
                                sys_drive, 
                                () -> DriveCommands.getRotation2d(
                                  sys_drive, 
                                  new Pose2d(
                                    new Translation2d(Hub.topCenterPoint.getMeasureX(), Hub.topCenterPoint.getMeasureY()), 
                                    Rotation2d.kZero
                                  )
                                )
                              )
                         );

        primaryController.leftBumper()
                        .whileTrue(
                          DriveCommands.joystickDriveAtAngle(
                            sys_drive,
                            () -> -primaryController.getLeftY(),
                            () -> -primaryController.getLeftX(),
                            () -> DriveCommands.getRotation2d(sys_drive, selectedPassingPosition.pose)
                          )
                        );

        primaryController.x()
                        .whileTrue(
                            Commands.sequence(
                              DriveCommands.alignToPoint(
                                sys_drive, 
                                () -> selectedClimibingPrepPosition.pose, 
                                () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY, 
                                () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION,
                                kAutoAlign.TRANSLATION_TOLERANCE_CLIMB_PREP,
                                kAutoAlign.ROTATION_TOLERANCE_CLIMB_PREP,
                                kAutoAlign.VELOCITY_TOLERANCE_CLIMB_PREP

                              ),
                              DriveCommands.alignToPoint(
                                sys_drive, 
                                () -> selectedClimbingPosition.pose, 
                                () -> kAutoAlign.MAX_AUTO_ALIGN_VELOCITY_CLIMB, 
                                () -> kAutoAlign.MAX_AUTO_ALIGN_ACCELERATION_CLIMB
                              )
                            )
                        );

        secondaryController.x()
                        .onTrue(prepPassingPositionCommand(PassingPositions.RIGHT));
        secondaryController.b()
                        .onTrue(prepPassingPositionCommand(PassingPositions.LEFT));
        secondaryController.a()
                        .onTrue(prepPassingPositionCommand(PassingPositions.MIDDLE));

        secondaryController.povLeft()
                        .onTrue(prepClimberPositionCommand(ClimbingPositions.LEFT));
        secondaryController.povRight()
                        .onTrue(prepClimberPositionCommand(ClimbingPositions.RIGHT));
  
        SmartDashboard.putData("extend", sys_intake.extend()); //TODO remove when main
        SmartDashboard.putData("retract", sys_intake.retract());
        SmartDashboard.putData("Start Roller", sys_intake.setRollerVoltage(12.0));
        SmartDashboard.putData("Stop Roller", sys_intake.setRollerVoltage(0.0));
    }

    private Command prepClimberPositionCommand(ClimbingPositions climbingPosition){
        return Commands.runOnce(
                () -> {
                        if (climbingPosition == ClimbingPositions.LEFT)
                          selectedClimibingPrepPosition = ClimbingPositions.LEFT_PREP;
                        else
                          selectedClimibingPrepPosition = ClimbingPositions.RIGHT_PREP;
                          
                        Logger.recordOutput("Climbing Position", climbingPosition);

                        selectedClimbingPosition = climbingPosition; 
                        
                        Logger.recordOutput("Climbing Selected Pose", selectedClimbingPosition.pose);

                }
        );
    };

    private Command prepPassingPositionCommand(PassingPositions passingPosition){
        return Commands.runOnce(
                () -> {
                        Logger.recordOutput("Passing Position", passingPosition);

                        selectedPassingPosition = passingPosition;

                        Logger.recordOutput("Passing Selected Pose", selectedPassingPosition.pose);

                }
        );
    }

    /** 
     * Command to extend both intake and hopper subsystems, with crash avoidance
     * @author Jaden Rajan, team 5409
     * @author John Chen, team 5409
     */
    private Command extendIntakeAndHopper() {
        return Commands.repeatingSequence(
                Commands.either(
                sys_intake.stopMotor(),
                sys_intake.extend(), 
                () -> sys_hopper.getPositionIntakeZero().minus(sys_intake.getPosition()).lt(IntakeConstants.Extension.KILLSWITCH_TOLERANCE) 
                        && !(sys_hopper.getPosition().isNear(HopperConstants.HOPPER_MAX_EXTENSION, HopperConstants.AGITATE_TOLERANCE))
                ).alongWith(sys_hopper.fullExtend())
        ).until(() -> {return sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_DISTANCE, HopperConstants.AGITATE_TOLERANCE);})
                .andThen(sys_hopper.fullExtend());
    }

    /** 
     * Command to retract both intake and hopper subsystems, with crash avoidance
     * @author Jaden Rajan, team 5409
     * @author John Chen, team 5409
     */
    private Command retractIntakeAndHopper() {
        return Commands.repeatingSequence(
                Commands.either(
                        sys_hopper.stopMotor(), 
                        sys_hopper.setSetpoint(() -> HopperConstants.HOPPER_MIN_EXTENSION), 
                        () -> sys_hopper.getPositionIntakeZero().minus(sys_intake.getPosition()).lt(IntakeConstants.Extension.KILLSWITCH_TOLERANCE) 
                        && !(sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, HopperConstants.AGITATE_TOLERANCE))
                ).alongWith(sys_intake.retract())
        ).until(() -> {return sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, HopperConstants.AGITATE_TOLERANCE);})
                .andThen(sys_hopper.fullRetract());
        
    }

    Distance intakeSetpoint;
    Distance hopperSetpoint;

    /** 
     * Command to retract both intake and hopper subsystems, while agitating hopper back and forth to help with launching fuel
     * @author Jaden Rajan, team 5409
     * @author John Chen, team 5409
     */
    private Command retractAndAgitate() {
        return Commands.repeatingSequence(
                Commands.runOnce(() -> 
                        intakeSetpoint = sys_intake.getPosition().minus((Inches.of(IntakeConstants.Extension.RETRACT_INCREMENT.in(Inches))))),
                Commands.runOnce(() -> 
                        hopperSetpoint = intakeSetpoint.plus(IntakeConstants.Extension.KILLSWITCH_TOLERANCE)
                                                        .minus(HopperConstants.STARTING_GAP_TO_INTAKE)),
                sys_intake.move(() -> intakeSetpoint),
                Commands.either(
                        sys_hopper.stopMotor(),
                        sys_hopper.setSetpoint(() -> hopperSetpoint),
                        () -> (sys_hopper.getPositionIntakeZero().minus(sys_intake.getPosition()))
                                        .lt(IntakeConstants.Extension.KILLSWITCH_TOLERANCE)
                ).repeatedly().until(() -> sys_hopper.getPosition().isNear(
                        hopperSetpoint, HopperConstants.AGITATE_TOLERANCE)),

                sys_hopper.setSetpoint(() -> hopperSetpoint.plus(HopperConstants.EXTEND_INCREMENT)),
                Commands.waitUntil(() -> 
                        sys_hopper.getPosition().isNear(hopperSetpoint.plus(HopperConstants.EXTEND_INCREMENT), 
                                                        HopperConstants.AGITATE_TOLERANCE))
        ).until(() -> sys_intake.getPosition().isNear(IntakeConstants.Extension.EXTENSION_MIN_DISTANCE, HopperConstants.AGITATE_TOLERANCE))
                .andThen(sys_hopper.fullRetract());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
