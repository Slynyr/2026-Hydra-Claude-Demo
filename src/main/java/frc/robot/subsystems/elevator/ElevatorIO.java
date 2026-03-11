package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
    @AutoLog
    class ElevatorInputs {
        public boolean  isConnected   = false;
        public Voltage  voltage       = Volts.of(0.0);
        public Current  supplyCurrent = Amps.of(0.0);
        public double   temperature   = 0.0; // Celsius
        public Distance position      = Meters.of(0.0);
        public Current  torqueCurrent = Amps.of(0.0);
    }

    default void updateInputs(ElevatorInputs inputs) {}

    default void setMotorVoltage(double voltage) {}

    default void stopMotor() {}

    default void zeroEncoder() {}

    default Distance getPosition() {
        return Meters.of(0.0);
    }

    default void setSetpoint(Distance setpoint, int slot) {}
}