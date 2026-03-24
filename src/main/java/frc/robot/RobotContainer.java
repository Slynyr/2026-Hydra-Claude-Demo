// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GameCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.launcher.*;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;
import frc.robot.subsystems.serializer.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.util.AutoPath;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Drive      sys_drive;
    public final Vision     sys_vision;
    public final Intake     sys_intake;
    public final Serializer sys_serializer;
    public final Feeder     sys_feeder;
    public final Hopper     sys_hopper;
    public final Launcher   sys_launcher;
    /**
     * THIS FIELD CAN BE NULL, ensure it is not-null before using it.
     */
    public final Elevator   sys_elevator;

    public static SwerveDriveSimulation simConfig;

    private ClimbingPositions selectedClimbingPosition     = ClimbingPositions.LEFT;
    private ClimbingPositions selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;

    private Distance manualLaunchDistance = Meters.of(2);

    public BooleanSupplier shouldLaunch = () -> true;

    public LoggedNetworkBoolean launcherShouldIdle = new LoggedNetworkBoolean("Launcher/ShouldIdle", false);

    // Controllers
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    public boolean aahanControls = false;

    private final Alert primaryDisconnectedAlert   = new Alert(
            "Primary Controller Disconnected!",
            AlertType.kError
    );
    private final Alert secondaryDisconnectedAlert = new Alert(
            "Secondary Controller Disconnected!",
            AlertType.kError
    );

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
                sys_intake = new Intake(
                        new IntakeIOTalonFX(DeviceID.INTAKE_ROLLER_MOTOR, DeviceID.INTAKE_EXTENSION_MOTOR));

                sys_serializer = new Serializer(
                        new SerializerIOTalonFX(DeviceID.SERIALIZER_MOTOR, DeviceID.FEEDER_MOTOR_BOTTOM));
                sys_feeder = new Feeder(new FeederIOTalonFX(DeviceID.FEEDER_MOTOR_TOP));
                sys_vision = new Vision(new VisionIOLimelight(DeviceID.LIMELIGHT_NAME));

                if (Constants.IS_CLIMBER_ATTACHED) {
                    sys_elevator = new Elevator(new ElevatorIOTalonFX(DeviceID.CLIMBER_MOTOR));
                } else {
                    System.out.println("Climber not attached, will not register");
                    sys_elevator = null;
                }

                sys_drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight),
                        sys_vision
                );

                sys_launcher = new Launcher(new LauncherIOTalonFX(
                        DeviceID.LAUNCHER_CANCODER,
                        DeviceID.LAUNCHER_MOTOR_1,
                        DeviceID.LAUNCHER_MOTOR_2,
                        DeviceID.LAUNCHER_HOOD_SERVO_1,
                        DeviceID.LAUNCHER_HOOD_SERVO_2),
                    sys_drive);
            }
            // Sim robot, instantiate physics sim IO implementations
            case SIM -> {
                final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig
                        .Default()
                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(DriveConstants.ROBOT_FULL_MASS)
                        .withTrackLengthTrackWidth(Meters.of(0.578), Meters.of(0.578))
                        .withBumperSize(Meters.of(0.881), Meters.of(0.881))
                        .withSwerveModule(
                                COTS.ofMark4i(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getKrakenX44(1),
                                        DriveConstants.WHEEL_COF,
                                        1));

                simConfig = new SwerveDriveSimulation(driveConfig, new Pose2d(3, 3, Rotation2d.kZero));

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
                        sys_vision);
                sys_elevator = new Elevator(new ElevatorIOSim());
                sys_intake = new Intake(new IntakeIOSim());
                sys_serializer = new Serializer(new SerializerIOSim());
                sys_feeder = new Feeder(new FeederIOSim());
                sys_hopper = new Hopper(new HopperIOSim());

                sys_launcher = new Launcher(new LauncherIOSim(), sys_drive);
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
                sys_launcher = new Launcher(new LauncherIO() {}, sys_drive);
            }
        }

        // Set up auto routines
        autoChooser = buildAutoChooser();
        buildLaunchStrategyChooser();

        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData(
            "Aahan Controls Set",
            Commands.runOnce(() -> {
                aahanControls = !aahanControls;
                Logger.recordOutput("Controls/AahanControls", aahanControls);
            }).ignoringDisable(true)
        );
        
        Logger.recordOutput("Controls/AahanControls", aahanControls);


        SmartDashboard.putData("Reset", Commands.runOnce(this::resetPose).ignoringDisable(true));

        new Trigger(() -> !primaryController.isConnected()).onChange(
                Commands.runOnce(() -> primaryDisconnectedAlert.set(!primaryController.isConnected()))
                        .ignoringDisable(true)
        );

        new Trigger(() -> !secondaryController.isConnected()).onChange(
                Commands.runOnce(() -> secondaryDisconnectedAlert.set(!secondaryController.isConnected()))
                        .ignoringDisable(true)
        );

        // When DS connects check joystick connections
        new Trigger(DriverStation::isDSAttached).onTrue(
                Commands.waitSeconds(1.0)
                        .andThen(
                                Commands.runOnce(() -> {
                                            primaryDisconnectedAlert.set(!primaryController.isConnected());
                                            secondaryDisconnectedAlert.set(!secondaryController.isConnected());

                                            resetPose();
                                        })
                                        .ignoringDisable(true)
                        )
        );

        new Trigger(() -> sys_launcher.getCurrentCommand() == null && DriverStation.isEnabled() && launcherShouldIdle.get())
                .onTrue(sys_launcher.runVelocity(() -> LauncherConstants.Launcher.LAUNCHER_IDLE_SPEED)
                    .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        new Trigger(() -> kField.NEUTRAL_ZONE.contains(sys_drive.getPose().getTranslation()))
                .onTrue(Commands.runOnce(() -> Logger.recordOutput("Drive/InNeutralZone", true)))
                .onFalse(Commands.runOnce(() -> Logger.recordOutput("Drive/InNeutralZone", false)));

        // TODO: When have time, test these 2 (Test if the robot can check if it is neutral zone or not)
//        new Trigger(() -> kField.NEUTRAL_ZONE.contains(sys_drive.getPose().getTranslation()))
//            .onTrue(Commands.runOnce(() -> shouldLaunch = () -> false))
//            .onFalse(Commands.runOnce(() -> shouldLaunch = () -> true));

        // TODO: test this, automatically idles launcher in neutral zone
//        new Trigger(() -> !kField.NEUTRAL_ZONE.contains(sys_drive.getPose().getTranslation()))
//            .whileTrue(sys_launcher.runVelocity(() -> LauncherConstants.Launcher.LAUNCHER_IDLE_SPEED)
//                                   .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    private void resetPose() {
        if (autoChooser.get() instanceof AutoPath path) {
            sys_drive.setPose(path.getStartingPose());
            if (Constants.CURRENT_MODE == Mode.SIM)
                simConfig.setSimulationWorldPose(path.getStartingPose());
        }
        if (autoChooser.get() instanceof PathPlannerAuto auto) {
            Pose2d pose = AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(auto.getStartingPose()) : auto.getStartingPose();
            sys_drive.setPose(pose);
            if (Constants.CURRENT_MODE == Mode.SIM)
                simConfig.setSimulationWorldPose(pose);
        }
    }

    /**
     * builds the dashboard command chooser ({@link LoggedDashboardChooser}) for picking launch strategies.
     *
     * @apiNote this chooser automatically schedules the chosen command.
     */
    public void buildLaunchStrategyChooser() {
        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Launch Strategy");

        for (LaunchStrategy strategy: LaunchStrategy.getLaunchStrategies())
            chooser.addOption(strategy.getName(), Commands.runOnce(() -> sys_launcher.setStrategy(strategy)));

        chooser.onChange(CommandScheduler.getInstance()::schedule);
    }

    /**
     * Updates sim positions of algae, coral and robot poses
     */
    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    private LoggedDashboardChooser<Command> buildAutoChooser() {
        LoggedDashboardChooser<Command> chooser = new LoggedDashboardChooser<>("Auto Choices");
        chooser.addDefaultOption("None", Commands.none());
        ArrayList<AutoPath> autoPaths = Autos.getAutoPaths(this);

        autoPaths.forEach(autoPath -> chooser.addOption(autoPath.getName(), autoPath));

        for (String auto: AutoBuilder.getAllAutoNames()) {
            chooser.addOption(auto, new PathPlannerAuto(auto));
        }

        if (Constants.IS_TUNING) {
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
        }

        chooser.onChange(_cmd -> resetPose());
        return chooser;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // TODO: if elastic can directly update a preference, remove this.
        SmartDashboard.putNumber("Launcher Speed Offset [rps]", Launcher.getSpeedOffset().in(RotationsPerSecond));

        SmartDashboard.putData(
                "Update Offset Now", Commands.runOnce(() -> Launcher.setSpeedOffset(
                        RotationsPerSecond.of(SmartDashboard.getNumber("Launcher Speed Offset [rps]", 0.0)))));

        SmartDashboard.putData(
                "Drive/Align Wheels 45",
                        Commands.runOnce(() -> sys_drive.runTurnSetpoint(Rotation2d.fromDegrees(45)))
                            .withTimeout(3.0));

        sys_drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        sys_drive,
                        () -> -primaryController.getLeftY(),
                        () -> -primaryController.getLeftX(),
                        () -> -(aahanControls
                                ? primaryController.getRightX()
                                : primaryController.getRightTriggerAxis() - primaryController.getLeftTriggerAxis())
                )
        );

        primaryController.start()
            .and(primaryController.back())
            .onTrue(
                Commands.runOnce(() -> sys_drive.setPose(new Pose2d(0, 0, Rotation2d.k180deg)))
                    .ignoringDisable(true)
            );

        primaryController.rightBumper()
            .whileTrue(
                    GameCommands.autoLaunch(
                        () -> DriveCommands.distToHub(sys_drive),
                        () -> -primaryController.getLeftY(),
                        () -> -primaryController.getLeftX(),
                        this
                    )
            )
            .onFalse(
                GameCommands.stopLaunching(this)
            );

        primaryController.y()
                        .onTrue(
                            GameCommands.startIntake(this)
                        );

        primaryController.leftBumper()
                        .whileTrue(GameCommands.manualPass(
                                () -> kField.RIGHT_HALF.contains(sys_drive.getPose().getTranslation()),
                                () -> -primaryController.getLeftY(),
                                () -> -primaryController.getLeftX(),
                                this));

        primaryController.x()
                        .onTrue(
                          GameCommands.retract(this)
                        );

        primaryController.b()
                        .onTrue(sys_intake.setRollerVoltage(0));

        primaryController.a()
                         .onTrue(Commands.runOnce(() -> DriveCommands.setTranslationSpeed(kBump.BUMP_SPEED_MODIFIER)))
                         .onFalse(Commands.runOnce(() -> DriveCommands.setTranslationSpeed(1.0)));

        primaryController.povDown()
                        .whileTrue(GameCommands.manualLaunch(() -> manualLaunchDistance, this))
                        .onFalse(GameCommands.stopLaunching(this));

        secondaryController.b()
                        .onTrue(GameCommands.retract(this));

        secondaryController.a()
                        .whileTrue(GameCommands.agitateThenRetract(this));

        // TODO: temp buttons to zero intake/hopper
        secondaryController.rightBumper()
                .onTrue(sys_intake.setExtensionVoltage(-2)
                            .alongWith(sys_hopper.setVoltage(-2)))
                .onFalse(sys_intake.setExtensionVoltage(0)
                            .alongWith(sys_hopper.setVoltage(0)));

        secondaryController.x()
                        .onTrue(sys_intake.setRollerVoltage(-IntakeConstants.Roller.INTAKE_VOLTAGE))
                        .onFalse(sys_intake.setRollerVoltage(IntakeConstants.Roller.INTAKE_VOLTAGE));


        secondaryController.povRight()
                        .onTrue(sys_serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE))
                        .onFalse(sys_serializer.setVoltage(0));

        secondaryController.y()
                        .onTrue(GameCommands.reverseRollers(this))
                        .onFalse(GameCommands.stopSerializing(this).alongWith(sys_feeder.setVoltage(0)));

        secondaryController.povLeft()
                        .onTrue(sys_serializer.setVoltage(-SerializerConstants.SERIALIZING_VOLTAGE))
                        .onFalse(sys_serializer.setVoltage(0));

        secondaryController.povUp()
                        .onTrue(Launcher.incrementSpeedOffset(RotationsPerSecond.of(1)));

        secondaryController.povDown()
                        .onTrue(Launcher.incrementSpeedOffset(RotationsPerSecond.of(-1)));

       secondaryController.leftTrigger()
               .onTrue(Commands.runOnce(sys_vision::captureClip));

        // TODO: GET MANUAL LAUNCH DISTANCE THAT WE WANT TO USE
        new Trigger(() -> secondaryController.getLeftX() > 0.5)
                    .onTrue(prepManualLaunchDistance(Meters.of(2.0)));

        // TODO: GET MANUAL LAUNCH DISTANCE THAT WE WANT TO USE
        new Trigger(() -> secondaryController.getLeftX() < 0.5)
                    .onTrue(prepManualLaunchDistance(Meters.of(4.0)));

        SmartDashboard.putNumber("Hood Angle [mm]", 0);
        SmartDashboard.putData("Set Hood Angle",
                               sys_launcher.setHoodExtension(() -> Millimeter.of(SmartDashboard.getNumber("Hood Angle [mm]", 0))));

        SmartDashboard.putData("Hopper/Coast", sys_hopper.coastMode().ignoringDisable(true)); //TODO remove when main
        SmartDashboard.putData("Hopper/Brake", sys_hopper.brakeMode().ignoringDisable(true)); //TODO remove when main

        SmartDashboard.putData("Intake/Coast", sys_intake.coastMode().ignoringDisable(true)); // TODO: REMOVE WHEN MAIN
        SmartDashboard.putData("Intake/Brake", sys_intake.brakeMode().ignoringDisable(true)); // TODO: REMOVE WHEN MAIN
    
        SmartDashboard.putData("Drive/Coast", Commands.runOnce(() -> sys_drive.coastMode()).ignoringDisable(true));
        SmartDashboard.putData("Drive/Brake", Commands.runOnce(() -> sys_drive.brakeMode()).ignoringDisable(false));

    }

    private Command prepClimberPositionCommand(ClimbingPositions climbingPosition) {
        return Commands.runOnce(
                () -> {
                    if (climbingPosition == ClimbingPositions.LEFT)
                        selectedClimbingPrepPosition = ClimbingPositions.LEFT_PREP;
                    else
                        selectedClimbingPrepPosition = ClimbingPositions.RIGHT_PREP;

                    Logger.recordOutput("ClimbingPosition", climbingPosition);

                    selectedClimbingPosition = climbingPosition;

                    Logger.recordOutput("ClimbingSelectedPose", selectedClimbingPosition.pose);
                }
        );
    }

    private Command prepManualLaunchDistance(Distance distance){
        return Commands.runOnce(
            () -> {
                manualLaunchDistance = distance;
                Logger.recordOutput("ManualLaunchDistance", manualLaunchDistance);
            }
        );
    }

    /**
     * Runs when {@link DriverStation#isDisabled()} changes to true. Disables all subsystem commands for safety
     * purposes.
     *
     * @return command that will run on disabled
     */
    public Command onDisable() {
        Command cmd = Commands.parallel(
                GameCommands.stopLaunching(this),
                Commands.runOnce(sys_drive::stop),
                sys_hopper.setVoltage(0)
        );

        if (sys_elevator != null)
            cmd = cmd.alongWith(sys_elevator.setVoltage(0));

        return cmd;
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
