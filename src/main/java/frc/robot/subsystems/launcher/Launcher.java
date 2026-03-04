package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.launcher.interpolator.LaunchConfig;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;
import frc.robot.util.MathUtils;
import frc.robot.utils.Checkmate;
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

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();

        // create the logged fields
        logInterpolation(Meters.of(0), null, RPM.of(0));
        setStrategy(LauncherConstants.Launcher.DEFAULT_LAUNCH_STRATEGY);
        Preferences.setDouble(PREF_LAUNCH_SPEED_OFFSET, getSpeedOffset().in(RotationsPerSecond));

        Checkmate.register(
                "Should launch fuel", () -> {
                    Distance d = Meters.of(2.0);
                    var config = strategy.interpolate(d);

                    // launch fuel with dummy IO for feeder; it doesn't matter if the feeder spins
                    CommandScheduler.getInstance().schedule(this.launchFuel(() -> d, new Feeder(new FeederIO() {})));

                    return MathUtils.withinTolerance(
                            getVelocity().in(RotationsPerSecond), config.speed().in(RotationsPerSecond), 0.05) ?
                           Checkmate.TestResult.success() :
                           Checkmate.TestResult.fail(
                                   "Launcher not fast enough (" + getVelocity().in(RotationsPerSecond) + " RPS)");
                });
    }

    public Command runVelocity(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> io.runVelocity(velocity));
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
     * Defers a command that interpolates a {@link LaunchConfig} and then sets the velocity and hood angle of the
     * launcher, based on the active {@link LaunchStrategy}.
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

                    return runVelocity(() -> launchSpeed)
                            .alongWith(setHoodAngle(c::angle)) // set hood angle
                            .alongWith(feeder.runRPS(() -> launchSpeed)); // run feeder at same vel.
                }, Set.of(this));
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
        Logger.recordOutput("Launcher/Interpolator/TargetAngle", config == null ? Radians.of(0) : config.angle());
        Logger.recordOutput("Launcher/Interpolator/RealLaunchSpeed", realLaunchSpeed);
    }

    private Distance computeHoodExtension(Angle angle) {
        // clamp between min and max
        double theta = angle.in(Degrees);
        theta = Math.max(LauncherConstants.Hood.MIN_ANGLE_DEG, Math.min(LauncherConstants.Hood.MAX_ANGLE_DEG, theta));

        return (Distance)Degrees.of(theta)
                                .timesConversionFactor(LauncherConstants.Hood.MM_PER_DEG)
                                .minus(LauncherConstants.Hood.OFFSET_MM);
    }

    public Command setHoodAngle(Supplier<Angle> angle) {
        return Commands.runOnce(() -> hoodSetpoint.set(computeHoodExtension(angle.get())));
    }

    // Getters
    public Angle getHoodAngle() {
        return Degrees.of(
                io.getHoodExtension()
                  .plus(LauncherConstants.Hood.OFFSET_MM)
                  .divideRatio(LauncherConstants.Hood.MM_PER_DEG)
                  .in(Degrees));
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    public Distance getUltrasonicDistance() {
        return io.getUltrasonicVolts().timesConversionFactor(LauncherConstants.Ultrasonic.MM_PER_VOLT);
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
        Logger.recordOutput("Launcher/Interpolator/OperatorSpeedOffset", getSpeedOffset());
        Logger.processInputs("Launcher", inputs);
        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);
    }
}
