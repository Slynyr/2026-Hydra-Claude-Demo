package frc.robot.subsystems.serializer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface SerializerIO {

    @AutoLog
    class SerializerInputs {
        public boolean         isSerializerConnected = false;
        public Voltage         serializerVoltage     = Volts.of(0.0);
        public Current         serializerCurrent     = Amps.of(0.0);
        public double          serializerTemperature = 0.0;
        public Angle           serializerPosition    = Degrees.of(0.0);
        public AngularVelocity serializerVelocity    = RotationsPerSecond.of(0.0);

        public Voltage targetVoltage = Volts.of(0);
    }

    default void updateInputs(SerializerInputs inputs) {}

    default void setVoltage(double voltage) {}

    default void stopMotors() {}

    default AngularVelocity getVelocity() {
        return RotationsPerSecond.of(0);
    }
}
