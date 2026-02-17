package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorInputs {
        public boolean isMainMotorConnected = false;
        public Voltage mainAppliedVoltage = Units.Volts.of(0.0);
        public Current mainAppliedCurrent = Units.Amps.of(0.0);
        public double mainMotorTemperature = 0.0; // Celsius
        public Distance mainMotorPosition = Units.Meters.of(0.0);
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

    public default void stopMotor() {}

    public default void zeroEncoder() {}

    public default Distance getPosition() {return Meters.of(0.0);}

    public default void setSetpoint(Distance setpoint) {}
    
}