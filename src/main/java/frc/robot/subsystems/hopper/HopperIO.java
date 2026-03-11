package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public interface HopperIO {
    @AutoLog
    class HopperInputs {
        public boolean  isConnected             = false;
        public Current  appliedCurrent          = Amps.of(0.0);
        public Current  torqueCurrent           = Amps.of(0.0);
        public Voltage  appliedVoltage   = Volts.of(0.0);
        public double   motorTemperature = 0.0;
        public Distance motorPosition    = Inches.of(0.0);
        public Distance setpoint                = Inches.of(0.0);
        public boolean  isCrashDetected         = false;
    }

    default void updateInputs(HopperInputs inputs) {}

    default void setMotorVoltage(double voltage) {}

    default void stopMotor() {}

    default void brakeMode() {}

    default void coastMode() {}

    default void setSetpoint(Supplier<Distance> setpoint) {}

    default Distance getPosition() {
        return Inches.of(0.0);
    }

    default Distance getSetpoint() {
        return Inches.of(0.0);
    }
}