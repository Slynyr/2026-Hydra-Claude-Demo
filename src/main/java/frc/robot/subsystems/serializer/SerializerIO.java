package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;


public interface SerializerIO {

    @AutoLog
    public class SerializerInputs {

        public boolean isIndexerMotorConnected = false;
        public Voltage indexerAppliedVoltage = Volts.of(0.0);
        public Current indexerAppliedCurrent = Amps.of(0.0);
        public double indexerMotorTemperature = 0.0;
        public Angle indexerMotorPosition = Degrees.of(0.0);
        public AngularVelocity indexerMotorVelocity = RotationsPerSecond.of(0.0);
        
        public boolean isBottomFeederMotorConnected = false;
        public Voltage bottomFeederAppliedVoltage = Volts.of(0.0);
        public Current bottomFeederAppliedCurrent = Amps.of(0.0);
        public double bottomFeederMotorTemperature = 0.0;
        public Angle bottomFeederMotorPosition = Degrees.of(0.0);
        public AngularVelocity bottomFeederMotorVelocity = RotationsPerSecond.of(0.0);
    }

    public default void updateInputs(SerializerInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default void stopMotors() {}
    
    public default void zeroEncoders() {}

    public default AngularVelocity getVelocity() {return RotationsPerSecond.of(0);}
}
