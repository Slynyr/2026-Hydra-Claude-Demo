package frc.robot.subsystems.launcher;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Launcher extends SubsystemBase {
    private final LauncherIO                io;
    private final LauncherInputsAutoLogged  inputs;
    private final AtomicReference<Distance> hoodSetpoint = new AtomicReference<>(Millimeters.of(0.0));

    private static final String PREF_LAUNCH_SPEED_OFFSET = "Launcher/SpeedOffsetRps";

    public Launcher(LauncherIO io) {
        this.io = io;
        inputs = new LauncherInputsAutoLogged();
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

    @Override
    public void periodic() {
        // update hood
        if (DriverStation.isEnabled()) {
            io.updateHood(hoodSetpoint.get());
        }

        // update inputs
        io.updateInputs(inputs);
        Logger.recordOutput("Launcher/Interpolator/OperatorSpeedOffset", getSpeedOffset());
        Logger.processInputs("Launcher", inputs);
        SmartDashboard.putData("Launcher/PID", LauncherConstants.Launcher.PID);
    }
}
