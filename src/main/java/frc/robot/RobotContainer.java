// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.kBump;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOSim;

import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerConstants;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.subsystems.serializer.SerializerIOSim;
import frc.robot.subsystems.serializer.SerializerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.subsystems.intake.IntakeConstants.Roller;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Meters;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    protected final   Drive  sys_drive;
    protected final   Vision sys_vision;
    protected final   Intake sys_intake;
    protected final   Serializer sys_serializer;


    public static SwerveDriveSimulation simConfig;

    // Controller
    private final CommandXboxController primaryController   = new CommandXboxController(0);
    private final CommandXboxController secondaryController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
                case REAL:
                        // Real robot, instantiate hardware IO implementations
                        sys_intake = new Intake(new IntakeIOTalonFX(Roller.MOTORID, Extension.MOTORID));
                        sys_serializer = new Serializer(new SerializerIOTalonFX(SerializerConstants.INDEXER_ID, SerializerConstants.FEEDER_ID));
                        sys_vision = new Vision(new VisionIOLimelight());

                        sys_drive =
                                new Drive(
                                        new GyroIOPigeon2(),
                                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                                        new ModuleIOTalonFX(TunerConstants.BackRight),
                                        sys_vision
                                );
                        
                        break;

                case SIM:
                        // Sim robot, instantiate physics sim IO implementations
                        sys_intake = new Intake(new IntakeIOSim());
                        sys_serializer = new Serializer(new SerializerIOSim());
                        
                        final DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default()
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

                        sys_drive =
                                new Drive(
                                        new GyroIOSim(simConfig.getGyroSimulation()),
                                        new ModuleIOSim(simConfig.getModules()[0]),
                                        new ModuleIOSim(simConfig.getModules()[1]),
                                        new ModuleIOSim(simConfig.getModules()[2]),
                                        new ModuleIOSim(simConfig.getModules()[3]),
                                        sys_vision
                                );
                        break;

                default:
                
                        sys_vision = new Vision(new VisionIO() {});
                        // Replayed robot, disable IO implementations
                        sys_drive = new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                sys_vision);
                                
                        sys_intake = new Intake(new IntakeIO(){});
                        sys_serializer = new Serializer(new SerializerIO() {});
                        break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(sys_drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(sys_drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                sys_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", sys_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", sys_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /*** Updates sim positions of algae, coral and robot poses
     */
    public void updateSim() {
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("Simulation/RobotPose", simConfig.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "Simulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
    
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
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
      .onTrue(
        Commands.runOnce(sys_drive::stopWithX, sys_drive));

    // Switch To Bump Speed Modifier
    primaryController.a()
      .onTrue(
        Commands.runOnce(() -> DriveCommands.setSpeed(kBump.BUMP_SPEED_MODIFIER)))
      .onFalse(
        Commands.runOnce(() -> DriveCommands.setSpeed(1.0)));
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
