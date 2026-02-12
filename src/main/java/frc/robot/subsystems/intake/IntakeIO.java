package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
        public double extensionPosition = 0.0;

        public boolean isExtended = false;
        public boolean isRetracted = true;

    }
    
    public default void setExtensionVoltage(double voltage) {}

    public default void setRollerVoltage(double voltage) {}

    public default void updateInputs(IntakeInputs inputs) {}

    public default Current getMotorCurrent() {
        return Amps.of(0.0);
    }

    public default void setSetpoint(Distance position) {}

    public default void coastMode() {}

    public default void brakeMode() {}

    public default void extend() {}

    public default void retract() {}

    public default void stopMotor() {}

    public default Distance getPosition() {  return null; }

    }

  
