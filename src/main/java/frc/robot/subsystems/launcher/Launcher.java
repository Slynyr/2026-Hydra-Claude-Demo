package frc.robot.subsystems.launcher;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GameCommandsConstants;
import frc.robot.Constants.kField;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.launcher.LauncherConstants.Hood;
import frc.robot.subsystems.launcher.interpolator.LaunchConfig;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;
import frc.robot.subsystems.serializer.Serializer;
import frc.robot.subsystems.serializer.SerializerConstants;
import frc.robot.subsystems.serializer.SerializerIO;
import frc.robot.util.Checkmate;
import frc.robot.util.MathUtils;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Launcher extends SubsystemBase {
    private final LauncherIO                io;
    private final LauncherInputsAutoLogged  inputs;
    private final AtomicReference<Distance> hoodSetpoint = new AtomicReference<>(Millimeters.of(0.0));

    private static final String PREF_LAUNCH_SPEED_OFFSET = "Launcher/SpeedOffsetRps";

    private LaunchStrategy strategy;
    private double         realLaunchSpeedRps;

    public Launcher(LauncherIO io, Drive drive) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        // create the logged fields
        logInterpolation(Meters.of(0), null, RPM.of(0));
        setStrategy(LauncherConstants.Launcher.DEFAULT_LAUNCH_STRATEGY);
        Preferences.setDouble(PREF_LAUNCH_SPEED_OFFSET, getSpeedOffset().in(RotationsPerSecond));

        Timer hoodInvalidationTimer = new Timer();
        hoodInvalidationTimer.start();

        // try to update the hood every 500 ms
        new Trigger(() -> hoodInvalidationTimer.advanceIfElapsed(Hood.HOOD_INVALIDATION_POLL_SECONDS))
                .onTrue(onHoodInvalidation(drive))
                .onFalse(Commands.runOnce(() -> Logger.recordOutput("Launcher/ShouldInvalidateHood", false)));

        Checkmate.register(
                "Should launch fuel", () -> {
                    Distance d = Meters.of(2.0);
                    var config = strategy.interpolate(d);

                    // launch fuel with dummy IO for feeder; it doesn't matter if the feeder spins
                    CommandScheduler.getInstance().schedule(this.launchFuel(
                            () -> d,
                            new Feeder(new FeederIO() {})
                    ));

                    return MathUtils.withinTolerance(
                            getVelocity().in(RotationsPerSecond), config.speed().in(RotationsPerSecond), 5) ?
                           Checkmate.TestResult.success() :
                           Checkmate.TestResult.fail(
                                   "Launcher not fast enough (" + getVelocity().in(RotationsPerSecond) + " RPS)");
                });
    }

    private Command onHoodInvalidation(Drive drive) {
        return Commands.runOnce(() -> {
            // outside neutral: automatic hood to hub
            if (!kField.NEUTRAL_ZONE.contains(drive.getPose().getTranslation())) {
                Logger.recordOutput("Launcher/ShouldInvalidateHood", true);
                Logger.recordOutput("Launcher/AutoHoodMode", "AllianceShoot");

                LaunchConfig launchEstimate = strategy.interpolate(DriveCommands.distToHub(drive));
                Logger.recordOutput(
                        "Launcher/HoodEstimateDifferential",
                        launchEstimate.hoodExtension().minus(hoodSetpoint.get()).abs(Millimeters));

                if (launchEstimate.hoodExtension().minus(hoodSetpoint.get()).abs(Millimeters) >=
                    Hood.HOOD_INVALIDATION_THRESHOLD_MM) {
                    // hoodSetpoint.set(launchEstimate.hoodExtension());
                    //TODO: debug cmd
                    hoodSetpoint.set(Millimeter.of(SmartDashboard.getNumber("Hood Angle [mm]", 0)));
                }
            }

            // inside neutral: set to passing
            else {
                hoodSetpoint.set(GameCommandsConstants.PASSING_HOOD_ANGLE);
                Logger.recordOutput("Launcher/AutoHoodMode", "NeutralPass");
            }
        });
    }

    public Command runVelocity(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> {
            realLaunchSpeedRps = velocity.get().in(RotationsPerSecond);
            io.runVelocity(velocity);
        });
    }

    /**
     * Gets the operator's launch speed offset [RPS] from the {@link Preferences} "Launcher/SpeedOffsetRps"
     *
     * @return double [RPS]
     */
    public static AngularVelocity getSpeedOffset() {
        return RotationsPerSecond.of(Preferences.getDouble(PREF_LAUNCH_SPEED_OFFSET, 0.0));
    }

    /**
     * Increments the operator's launch speed offset, which changes the real output speed sent to the launcher/feeder
     * when trying to launch fuel.
     *
     * @param by amount to increment offset by. can be a negative number to decrement.
     *
     * @see Launcher#setSpeedOffset(AngularVelocity) set/replace the constant instead
     */
    public static Command incrementSpeedOffset(AngularVelocity by) {
        return Commands.runOnce(() -> setSpeedOffset(getSpeedOffset().plus(by)));
    }

    /**
     * Sets the operator's launch speed offset, which changes the real output speed sent to the launcher/feeder when
     * trying to launch fuel.
     *
     * @param to amount to set the offset to
     *
     * @see Launcher#incrementSpeedOffset(AngularVelocity) increment the constant instead
     */
    public static void setSpeedOffset(AngularVelocity to) {
        var speed = to.in(RotationsPerSecond);
        Preferences.setDouble(PREF_LAUNCH_SPEED_OFFSET, speed);
        SmartDashboard.getEntry(PREF_LAUNCH_SPEED_OFFSET).setDouble(speed);
    }

    /**
     * Checks if the launcher's roller is at or near the target launch speed, within a 5% tolerance.
     *
     * @return true or false
     */
    public boolean isLauncherAtSpeed() {
        return MathUtils.withinTolerance(getVelocity().in(RotationsPerSecond), realLaunchSpeedRps, 5);
    }

    /**
     * Defers a command that interpolates a {@link LaunchConfig} and then sets the velocity and hood hoodExtension of
     * the launcher, based on the active {@link LaunchStrategy}.
     *
     * @param distance supplier to get the distance that fuel should be shot from
     *
     * @return deferred command that launches fuel
     */
    public Command launchFuel(Supplier<Distance> distance, Feeder feeder) {
        return Commands.defer(
                () -> {
                    LaunchConfig c = strategy.interpolate(distance.get());
                    AngularVelocity launchSpeed = c.speed().plus(getSpeedOffset());
                    logInterpolation(distance.get(), c, launchSpeed);

                    return startLaunchSequence(() -> launchSpeed, c::hoodExtension, feeder);
                }, Set.of(this));
    }

    public Command startLaunchSequence(Supplier<AngularVelocity> launchSpeed, Supplier<Distance> hoodExt, Feeder feeder) {
        return runVelocity(launchSpeed)
                .alongWith(setHoodExtension(hoodExt)) // set hood hoodExtension
                .alongWith(feeder.setUpperFeederVelocity(this::calculateUpperFeederVelocity)); // run upper feeder at same vel.
    }

    public Command serializeFuel(Feeder feeder, Serializer serializer) {
        // TODO: if the lower feeder is too slow, hardcode this to a faster number (i.e. 20 000 RPM)
        return feeder.setLowerFeederVelocity(
                        () -> calculateLowerFeederVelocity(this.getSurfaceVelocity(), serializer.getBeltSpeed()))
                     .alongWith(serializer.setVoltage(SerializerConstants.SERIALIZING_VOLTAGE));
    }

    // TODO: try setting upper feeder to the same as lower feeder velocities (smooth accerlation)
    @AutoLogOutput(key = "Launcher/CalculatedUpperFeederVelocity")
    private AngularVelocity calculateUpperFeederVelocity() {
        // overshoot of 7.5%
        return MathUtils.calculateAngularVelocity(getSurfaceVelocity().times(1.075), FeederConstants.FEEDER_ROLLER_CIRCUMFERENCE);
    }

    @AutoLogOutput(key = "Launcher/CalculatedLowerFeederVelocity")
    private AngularVelocity calculateLowerFeederVelocity(LinearVelocity launcherRollerSpeed, LinearVelocity serializerBeltSpeed) {
        return MathUtils.calculateAngularVelocity(
                launcherRollerSpeed.plus(MetersPerSecond.of(25)).div(2),
                FeederConstants.FEEDER_ROLLER_CIRCUMFERENCE);
    }

    /**
     * Updates the real outputs for the launcher interpolation system
     *
     * @param distance        distance from hub used in interpolation
     * @param config          resulting {@link LaunchConfig} from the {@link Launcher#strategy}
     * @param realLaunchSpeed actual launch speed sent to launcher/feeder, being the interpolated speed plus the
     *                        operator's manual offset
     */
    private void logInterpolation(Distance distance, LaunchConfig config, AngularVelocity realLaunchSpeed) {
        Logger.recordOutput("Launcher/Interpolator/TargetDistance", distance);
        Logger.recordOutput("Launcher/Interpolator/DidInterpolationSucceed", config != null);
        Logger.recordOutput(
                "Launcher/Interpolator/TargetSpeed",
                config == null ? RotationsPerSecond.of(0) : config.speed());
        Logger.recordOutput(
                "Launcher/Interpolator/TargetAngle",
                config == null ? Millimeters.of(0) : config.hoodExtension());
        realLaunchSpeedRps = realLaunchSpeed.in(RotationsPerSecond);
    }

    public Command setHoodExtension(Supplier<Distance> angle) {
        return Commands.runOnce(() -> hoodSetpoint.set(angle.get()));
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    @AutoLogOutput(key = "Launcher/SurfaceVelocity", unit = "m/s")
    public LinearVelocity getSurfaceVelocity() {
        return MathUtils.calculateSurfaceSpeed(getVelocity(), LauncherConstants.Launcher.ROLLER_CIRCUMFERENCE);
    }

    // Stops
    public Command stopLauncher() {
        return Commands.runOnce(io::stopLauncher, this);
    }

    public void setStrategy(LaunchStrategy strategy) {
        this.strategy = strategy;
        this.strategy.setLauncher(this);
        Logger.recordOutput("Launcher/Interpolator/LaunchStrategy", strategy.getName());
    }

    @Override
    public void periodic() {
        // update hood
        if (DriverStation.isEnabled()) {
            io.updateHood(hoodSetpoint.get());
            strategy.periodic();
        }

        // update inputs
        io.updateInputs(inputs);
        Logger.recordOutput("Components/Hood", new Pose3d());
        Logger.recordOutput("Launcher/Interpolator/OperatorSpeedOffset", getSpeedOffset());
        Logger.recordOutput("Launcher/IsAtSpeed", isLauncherAtSpeed());
        Logger.recordOutput("Launcher/RealLaunchSpeed", RotationsPerSecond.of(realLaunchSpeedRps));
        Logger.processInputs("Launcher", inputs);
    }
}
