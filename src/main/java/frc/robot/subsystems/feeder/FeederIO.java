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
        public boolean         isUpperMotorConnected = false;
        public Voltage         upperVoltage          = Volts.of(0.0);
        public Current         upperCurrent          = Amps.of(0.0);
        public double          upperTemperature      = 0.0;
        public Angle           upperPosition         = Degrees.of(0.0);
        public AngularVelocity upperVelocity         = RotationsPerSecond.of(0.0);
        public AngularVelocity upperSetpoint         = RotationsPerSecond.of(0.0);

        public boolean         isLowerMotorConnected = false;
        public Voltage         lowerVoltage          = Volts.of(0.0);
        public Current         lowerCurrent          = Amps.of(0.0);
        public double          lowerTemperature      = 0.0;
        public Angle           lowerPosition         = Degrees.of(0.0);
        public AngularVelocity lowerVelocity         = RotationsPerSecond.of(0.0);
        public AngularVelocity lowerSetpoint         = RotationsPerSecond.of(0.0);
    }

    default void updateInputs(FeederInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void setUpperFeederVelocity(Supplier<AngularVelocity> velocity) {}

    default void setLowerFeederVelocity(Supplier<AngularVelocity> velocity) {}

    default void stopMotors() {}

    default AngularVelocity getUpperFeederVelocity() {
        return RotationsPerSecond.of(0);
    }

    default AngularVelocity getLowerFeederVelocity() {
        return RotationsPerSecond.of(0);
    }
}