package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {

    @AutoLog
    public class IntakeInputs{

        public boolean isRollerConnected = false;
        public Voltage rollerVolts = Volts.of(0.0);
        public Current rollerCurrent = Amps.of(0.0);
        public double rollerTemp = 0.0;
        public AngularVelocity rollerVelocity = RotationsPerSecond.of(0.0);

        public boolean isExtensionConnected = false;
        public boolean isExtensionRunning = false;
        public Voltage extensionVolts = Volts.of(0.0);
        public Current extensionCurrent = Amps.of(0.0);
        public double extensionTemp = 0.0;
        public LinearVelocity extensionVelocity = MetersPerSecond.of(0.0);
        public Distance extensionPosition = Meters.of(0.0);
        public Current extensionTorqueCurrent = Amps.of(0.0);
        public Distance extensionSetpoint = Meters.of(0.0);

        public boolean isExtended = false;
        public boolean isRetracted = true;
        public boolean isCrashDetected = false;

    }
/**
* Sets voltage of extension
* @param voltage The voltage to set the extension motor to, in volts. Should be between -12 and 12.
 */
    public default void setExtensionVoltage(double voltage) {}
/**
* Sets voltage of roller
* @param voltage The voltage to set the roller motor to, in volts. Should be between -12 and 12.
 */
    public default void setRollerVoltage(double voltage) {}
/**
* Updates inputs
* @param inputs The inputs object to update with the latest sensor values and other relevant information.
 */
    public default void updateInputs(IntakeInputs inputs) {}
/**
* Gets current of motor
* @return The current drawn by the extension motor, in amps.
 */
    public default Current getMotorCurrent() {
        return Amps.of(0.0);
    }
/**
* Moves intake to setpoint
* @param position The position to set the extension motor to, in meters.
 */
    public default void setSetpoint(Distance position) {}
/**
* Sets both motors to coastMode
* @return A command that sets the extension motor to coast mode when executed.
 */
    public default void coastMode() {}
/**
* Sets both motors to brakeMode
* @return A command that sets the extension motor to brake mode when executed.
 */
    public default void brakeMode() {}
/**
* Extends the intake 
* @return A command that extends the intake when executed.
 */
    public default void extend() {}
/**
* Retracts the intake
* @return A command that retracts the intake when executed.
 */
    public default void retract() {}
/**
* Stops the motor
* @return A command that stops the extension motor when executed.
 */
    public default void stopMotor() {}
/**
* Gets the position of the intake extension
* @return The current position of the intake extension, in meters.
 */
    public default Distance getPosition() {  return null; }

    }

  
