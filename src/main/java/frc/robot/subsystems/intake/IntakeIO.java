package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {

    @AutoLog
    class IntakeInputs {
        public boolean isRollerConnected         = false;
        public boolean isRollerFollowerConnected = false;

        public Voltage         rollerVolts    = Volts.of(0.0);
        public Current         rollerCurrent  = Amps.of(0.0);
        public Current         rollerStatorCurrent  = Amps.of(0.0);
        public double          rollerTemp     = 0.0;
        public AngularVelocity rollerVelocity = RotationsPerSecond.of(0.0);

        public Voltage         rollerFollowerVolts    = Volts.of(0.0);
        public Current         rollerFollowerCurrent  = Amps.of(0.0);
        public Current         rollerFollowerStatorCurrent  = Amps.of(0.0);
        public double          rollerFollowerTemp     = 0.0;
        public AngularVelocity rollerFollowerVelocity = RotationsPerSecond.of(0.0);

        public boolean        isExtensionConnected   = false;
        public boolean        isExtensionRunning     = false;
        public Voltage        extensionVolts         = Volts.of(0.0);
        public Current        extensionCurrent       = Amps.of(0.0);
        public Current        extensionStatorCurrent       = Amps.of(0.0);
        public double         extensionTemp          = 0.0;
        public LinearVelocity extensionVelocity      = MetersPerSecond.of(0.0);
        public Distance       extensionPosition      = Meters.of(0.0);
        public Current        extensionTorqueCurrent = Amps.of(0.0);
        public Distance       extensionSetpoint      = Meters.of(0.0);

        public boolean isCrashDetected = false;
    }

    /**
     * Sets voltage of extension
     *
     * @param voltage The voltage to set the extension motor to, in volts. Should be between -12 and 12.
     */
    default void setExtensionVoltage(double voltage) {}

    /**
     * Sets voltage of roller
     *
     * @param voltage The voltage to set the roller motor to, in volts. Should be between -12 and 12.
     */
    default void setRollerVoltage(double voltage) {}

    /**
     * Updates inputs
     *
     * @param inputs The inputs object to update with the latest sensor values and other relevant information.
     */
    default void updateInputs(IntakeInputs inputs) {}

    /**
     * Moves intake to setpoint
     *
     * @param position The position to set the extension motor to, in meters.
     */
    default void setSetpoint(Distance position) {}

    default Distance getSetpoint() {
        return Meters.of(0);
    }

    /**
     * Sets both motors to coastMode
     */
    default void coastMode() {}

    /**
     * Sets both motors to brakeMode
     */
    default void brakeMode() {}

    /**
     * Stops the motor
     */
    default void stopMotor() {}

    /**
     * Gets the position of the intake extension
     *
     * @return The current position of the intake extension, in meters.
     */
    default Distance getPosition() {
        return null;
    }

    default void zeroExtension() {}
}

  
