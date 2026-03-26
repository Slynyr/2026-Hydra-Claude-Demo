// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.PassingPositions;
import frc.robot.Constants.kAutoAlign;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AlignHelper;
import frc.robot.util.MathUtils;
import frc.robot.util.ProfiledController;

import static edu.wpi.first.units.Units.*;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double TRIGGER_DEADBAND = 0.01;
  private static final double ANGLE_KP = 5.0; // 7
  private static final double ANGLE_KD = 0.1; // 0.4
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec  Last year -> 1.0
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static double translationSpeedModifier = 0.6;
  private static double rotationSpeedModifier = 0.5;

  private static boolean isAligned = false;

  /**
   * used for the {@link DriveCommands#crossBump(Drive, Vision, Supplier, Supplier, Time)} command to only start counting
   * the timer once the robot initially leaves the ground
  */
  public static final AtomicBoolean DID_GET_OFF_GROUND = new AtomicBoolean();

  private static AtomicLong lastTime = new AtomicLong(System.currentTimeMillis());


  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  // Increase drive translation speed
  public static Command setRotationSpeedHigh(Drive drive) {
    return Commands.run(
            () -> rotationSpeedModifier = 1.0);
  }

  // Decrease drive translation speed
  public static Command setRotationSpeedLow(Drive drive) {
    return Commands.run(
            () -> rotationSpeedModifier = 0.5);
    }

  public static void setRotationSpeed(double speed){
    rotationSpeedModifier = speed;
  }

  public static double getRotationSpeed(){
    return rotationSpeedModifier;
  }

  // Increase drive translation speed
  public static Command setTranslationSpeedHigh(Drive drive) {
    return Commands.run(
            () -> translationSpeedModifier = 1.0);
  }

  // Decrease drive translation speed
  public static Command setTranslationSpeedLow(Drive drive) {
    return Commands.run(
            () -> translationSpeedModifier = 0.5);
    }

  public static void setTranslationSpeed(double speed){
    translationSpeedModifier = speed;
  }

  public static double getTranslationSpeed(){
    return translationSpeedModifier;
  }

  public static boolean isAligned(){
    return isAligned;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), TRIGGER_DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          if (Math.abs(omega) >= TRIGGER_DEADBAND)
            omega += Math.copySign(0.05, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * translationSpeedModifier,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * translationSpeedModifier,
                  omega * drive.getMaxAngularSpeedRadPerSec() * rotationSpeedModifier);

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an hoodExtension, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              Angle difference = AlignHelper.rotationDifference(drive.getRotation(), rotationSupplier.get());

              if (difference.lte(kAutoAlign.ROTATION_TOLERANCE))
                  omega = 0;
              

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * translationSpeedModifier,
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * translationSpeedModifier,
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));

              Logger.recordOutput("AutoAlign/TargetRotation", rotationSupplier.get());
              Logger.recordOutput("AutoAlign/OmegaOutput", omega);
              Logger.recordOutput("AutoAlign/MaxVelocity [Rotations per s]", RotationsPerSecond.of(ANGLE_MAX_VELOCITY));
              Logger.recordOutput("AutoAlign/MaxAcceleration [Rotations per s^2]", RotationsPerSecondPerSecond.of(ANGLE_MAX_ACCELERATION));
              Logger.recordOutput("AutoAlign/Angle to Alignment [Degrees]", difference.in(Degrees));

//              TODO: TUNE THIS PERCENTAGE TO BE ABLE TO LAUNCH FASTER
//              if (MathUtil.isNear(Degrees.of(drive.getRotation().getDegrees()), Degrees.of(rotationSupplier.get().getDegrees()), 1.5))
              if (MathUtils.isAngleNear(drive.getRotation(), rotationSupplier.get(), Degrees.of(1.5)))
                  isAligned = true;
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static Command alignToPoint(Drive drive, Supplier<Pose2d> target, Supplier<LinearVelocity> maxVelocity, Supplier<LinearAcceleration> maxAcceleration ){
    return alignToPoint(drive, target, maxVelocity, maxAcceleration, kAutoAlign.TRANSLATION_TOLERANCE, kAutoAlign.ROTATION_TOLERANCE, kAutoAlign.VELOCITY_TOLERANCE);
  }

  @SuppressWarnings("resource")
  public static Command alignToPoint(Drive drive, Supplier<Pose2d> target, Supplier<LinearVelocity> maxVelocity, Supplier<LinearAcceleration> maxAcceleration, Distance translationTolerance, Angle rotationTolerance, LinearVelocity velocityTolerance ){
    ProfiledController translationController = 
        new ProfiledController(
          kAutoAlign.ALIGN_PID,
          maxVelocity.get().in(MetersPerSecond),
          maxAcceleration.get().in(MetersPerSecondPerSecond)
        );

    PIDController headingController = 
        new PIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD
        );

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.sequence(
      Commands.runOnce(() -> {
          isAligned = false;

          ChassisSpeeds speeds = drive.getFieldRelativeSpeeds();

          translationController.reset(-Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
          headingController.reset();
      }),
      Commands.run(() -> {

        translationController.setContraints(maxVelocity.get().in(MetersPerSecond), maxAcceleration.get().in(MetersPerSecondPerSecond));

        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = target.get();

        if (AutoBuilder.shouldFlip()){
          targetPose = FlippingUtil.flipFieldPose(targetPose);
        }

        double xDiff = targetPose.getX() - robotPose.getX();
        double yDiff = targetPose.getY() - robotPose.getY();

        double totalDiff = Math.hypot(xDiff, yDiff);

        double speed = Math.abs(translationController.calculate(totalDiff, 0.0));

        double omega = 
            headingController.calculate(
              robotPose.getRotation().getRadians(), targetPose.getRotation().getRadians()
            );

        double speedX = speed * (xDiff/totalDiff);
        double speedY = speed * (yDiff/totalDiff);

        drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, omega, drive.getRotation()));
        
        
        Logger.recordOutput("AutoAlign/Target", targetPose);
        Logger.recordOutput("AutoAlign/SpeedOutput", speed);
        Logger.recordOutput("AutoAlign/OmegaOutput", omega);
        Logger.recordOutput("AutoAlign/MaxVelocity [m per s]", maxVelocity.get());
        Logger.recordOutput("AutoAlign/MaxAcceleration [m per s^2]", maxAcceleration.get());

      }, drive)
    ).until(() -> {
        Pose2d robotPose = drive.getPose();
        Pose2d targetPose = target.get();
        if(AutoBuilder.shouldFlip())
            targetPose =  FlippingUtil.flipFieldPose(targetPose);

        Angle difference = AlignHelper.rotationDifference(targetPose.getRotation(), robotPose.getRotation());

        Distance distance = Meters.of(Math.hypot(robotPose.getX() - targetPose.getX(), robotPose.getY() - targetPose.getY()));

        ChassisSpeeds speeds = drive.getChassisSpeeds();
        LinearVelocity robotSpeed = MetersPerSecond.of(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

        Logger.recordOutput("AutoAlign/Distance To Alignment [cm]", distance.in(Centimeters));
        Logger.recordOutput("AutoAlign/Angle To Alignment [degrees]", difference.in(Degrees));
        Logger.recordOutput("AutoAlign/Velocity [m per s]", robotSpeed.in(MetersPerSecond));

        
        return 		distance.lte(translationTolerance)
                && 	difference.lte(rotationTolerance)
                && 	robotSpeed.lte(velocityTolerance);

      }
    ).andThen(
        Commands.runOnce(() -> {
          drive.stop();
          isAligned = true;
        }, drive)
    );
  }

  @SuppressWarnings("resource")
  public static Command alignToHeading(Drive drive, Supplier<Rotation2d> target){
    PIDController headingController =
      new PIDController(
        ANGLE_KP, 
        0,
        ANGLE_KD);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.sequence(
      Commands.runOnce(() -> {

        isAligned = false;

        headingController.reset();
      }),
      Commands.run(() -> {
        
        Rotation2d robotRotation = drive.getRotation();
        Rotation2d targetRotation = target.get();

        double omega = 
          headingController.calculate(robotRotation.getRadians(), targetRotation.getRadians());
        
        if (Math.abs(omega) < 0.2){ // some tuned value
            omega = 0;
            isAligned = true;
        } else {
            isAligned = false;
        }

        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(0.0,0.0, omega, drive.getRotation()));

        Logger.recordOutput("AutoAlign/TargetRotation", targetRotation);
        Logger.recordOutput("AutoAlign/OmegaOutput", omega);
        Logger.recordOutput("AutoAlign/isAligned", isAligned);

      }, drive)
    );
    // .until(() -> {
    //     Rotation2d robotRotation = drive.getRotation();
    //     Rotation2d targetRotation = target.get();

    //     Angle difference = AlignHelper.rotationDifference(targetRotation, robotRotation);

    //     AngularVelocity rotationSpeed = RadiansPerSecond.of(drive.getChassisSpeeds().omegaRadiansPerSecond);

    //     Logger.recordOutput("AutoAlign/Angle To Alignment [degrees]", difference.in(Degrees));
    //     Logger.recordOutput("AutoAlign/Velocity [degrees per s]", rotationSpeed.in(DegreesPerSecond));

    //     if (difference.lte(kAutoAlign.ROTATION_TOLERANCE)
    //             && rotationSpeed.lte(kAutoAlign.ROTATION_VELOCITY_TOLERANCE))
    //         alignedCounter[0]++;
        

    //     return difference.lte(kAutoAlign.ROTATION_TOLERANCE)
    //             && rotationSpeed.lte(kAutoAlign.ROTATION_VELOCITY_TOLERANCE);

    // })
    // .andThen(
    //     Commands.runOnce(() -> {
    //       drive.stop();
    //       isAligned = true;
    //     }, drive)
    // );
  }

  public static Command crossBump(Drive drive, Vision vision, Supplier<Rotation2d> targetHeading, Supplier<LinearVelocity> speed, Time timeout){
    //  -----------------SIM----------------
    if (Constants.CURRENT_MODE == Mode.SIM)
        return Commands.sequence(
            // DriveCommands.alignToHeading(
            //     drive, 
            //     targetHeading
            // ),
            Commands.run(() -> drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(
                        speed.get(),
                        MetersPerSecond.of(0.0),
                        RadiansPerSecond.of(0.0)
                ),
                drive.getRotation())
            ), drive).withTimeout(2),
            Commands.runOnce(drive::stop)
        );

    // ---------- REAL -----------
    return Commands.sequence(
        // DriveCommands.alignToHeading(
        //     drive, 
        //     targetHeading
        // ),
        Commands.runOnce(() -> {
            DID_GET_OFF_GROUND.set(false);
            lastTime.set(-1);
        }),
        Commands.run(() -> drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    speed.get(),
                    MetersPerSecond.of(0.0),
                    RadiansPerSecond.of(0.0)
                ),
                drive.getRotation())
        ), drive)
        .until(() -> {
            if (drive.getTilt().gt(Degrees.of(2.5)) && !DID_GET_OFF_GROUND.get())
                DID_GET_OFF_GROUND.set(true);

            if (!DID_GET_OFF_GROUND.get()) return false;

            // TODO: Tune for real robot value
            if (drive.getTilt().gt(Degrees.of(2.0)) && lastTime.get() == -1){ // should be much lower on real robot
            //     lastTime.set(-2);
            // }

            // if (drive.getTilt().lte(Degrees.of(2.0)) && lastTime.get() == -2) {
                lastTime.set(System.currentTimeMillis());
            }

            double dt = System.currentTimeMillis() - lastTime.get();
            Logger.recordOutput("Drive/BumpTimer", dt);
            Logger.recordOutput("Drive/BumpTimeout", timeout.in(Millisecond));


            return (
                drive.getTilt().lte(Degrees.of(3.5)) 
                && 
                (
                    dt > timeout.in(Millisecond) 
            // || vision.hasTarget()
            )
            );
        }),
        Commands.runOnce(drive::stop)
    );
  }

  public static Command crossBumpDeadline(Drive drive, Supplier<LinearVelocity> speed){
    return Commands.deadline(
        Commands.sequence(
            // Robot starts on bump
            Commands.waitUntil(() -> drive.getTilt().gte(Degrees.of(3))),
            // Reaches top of bump
            Commands.waitUntil(() -> drive.getTilt().lte(Degrees.of(3))),
            // Reaches flat ground again
            Commands.waitUntil(() -> drive.getTilt().lte(Degrees.of(3)))
        ), 
        Commands.run(() -> drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                new ChassisSpeeds(
                    speed.get(),
                    MetersPerSecond.of(0.0),
                    RadiansPerSecond.of(0.0)
                ),
                drive.getRotation())
        ), drive)
    ).andThen(
        drive::stop,
        drive
    );
  }

  public static Distance distToHub(Drive drive){
    Pose2d hubPose = Constants.kField.BLUE_HUB;
   if (AutoBuilder.shouldFlip())
       hubPose = Constants.kField.RED_HUB;

    return Meters.of((drive.getPose().getTranslation().getDistance(hubPose.getTranslation())));
  }

  /*
   * Gets Rotation2d to target pose from drive pose
   */
  public static Rotation2d getRotation2d(Drive drive, Pose2d targetPose){

    if (AutoBuilder.shouldFlip())
      targetPose = FlippingUtil.flipFieldPose(targetPose);

    return new Rotation2d(
      targetPose.getX() - drive.getPose().getX(),
      targetPose.getY() - drive.getPose().getY()
    );
  }

  public static Rotation2d getRotationToHub(Drive drive){
    return getRotation2d(drive, Constants.kField.BLUE_HUB).plus(Rotation2d.k180deg);
  }

  public static Rotation2d getRotationToPassingPosition(Drive drive, BooleanSupplier isRightHalf){
    return getRotation2d(drive, isRightHalf.getAsBoolean() 
                    ? AutoBuilder.shouldFlip() ? PassingPositions.LEFT.getPose() : PassingPositions.RIGHT.getPose() 
                    : AutoBuilder.shouldFlip() ? PassingPositions.RIGHT.getPose() : PassingPositions.LEFT.getPose() ).plus(Rotation2d.k180deg);
  }

  public static LinearVelocity getBumpSpeed(LinearVelocity speed) {
    if (AutoBuilder.shouldFlip())
        return speed.times(-1);
    return speed; 
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(() -> {
            velocitySamples.clear();
            voltageSamples.clear();
        }),

        // Allow modules to orient
        Commands.run(() -> {
            drive.runCharacterization(0.0);
        }, drive)
        .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(() -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
            }, drive)

            // When cancelled, calculate and print results
            .finallyDo(() -> {
                int n = velocitySamples.size();
                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;
                for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }
                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));
            }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> {
                limiter.reset(0.0);
            }),

            // Turn in place, accelerating up to full speed
            Commands.run(() -> {
                double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
            }, drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(() -> {
                state.positions = drive.getWheelRadiusCharacterizationPositions();
                state.lastAngle = drive.getRotation();
                state.gyroDelta = 0.0;
            }),

            // Update gyro delta
            Commands.run(() -> {
                var rotation = drive.getRotation();
                state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                state.lastAngle = rotation;
            })

            // When cancelled, calculate and print results
            .finallyDo(() -> {
                double[] positions = drive.getWheelRadiusCharacterizationPositions();
                double wheelDelta = 0.0;
                for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                }
                double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                NumberFormat formatter = new DecimalFormat("#0.000");
                System.out.println(
                    "********** Wheel Radius Characterization Results **********");
                System.out.println(
                    "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                System.out.println(
                    "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                System.out.println(
                    "\tWheel Radius: "
                    + formatter.format(wheelRadius)
                    + " meters, "
                    + formatter.format(Units.metersToInches(wheelRadius))
                    + " inches");
            })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
