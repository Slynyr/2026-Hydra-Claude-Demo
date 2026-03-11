package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public interface FeederIO {
    @AutoLog
    class FeederInputs {
        public boolean isMotorConnected = false;

        public Voltage         voltage     = Volts.of(0.0);
        public Current         current     = Amps.of(0.0);
        public double          temperature = 0.0;
        public Angle           position    = Degrees.of(0.0);
        public AngularVelocity velocity    = RotationsPerSecond.of(0.0);
        public AngularVelocity setpoint    = RotationsPerSecond.of(0.0);
    }

    default void updateInputs(FeederInputs inputs) {}

    default void setMotorVoltage(double voltage) {}

    default void runVelocity(Supplier<AngularVelocity> velocity) {}

    default void stopMotor() {}

    default AngularVelocity getVelocity() {
        return RotationsPerSecond.of(0);
    }
}