package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
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
        public AngularVelocity indexerMotorVelocity = DegreesPerSecond.of(0.0);
        
        public boolean isFeederMotorConnected = false;
        public Voltage feederAppliedVoltage = Volts.of(0.0);
        public Current feederAppliedCurrent = Amps.of(0.0);
        public double feederMotorTemperature = 0.0;
        public Angle feederMotorPosition = Degrees.of(0.0);
        public AngularVelocity feederMotorVelocity = DegreesPerSecond.of(0.0);
    }

    public default void updateInputs(SerializerInputs inputs) {}

    public default void setIndexerMotorVoltage(double voltage) {}
    public default void setFeederMotorVoltage(double voltage) {}

    public default void stopIndexerMotor() {}
    public default void stopFeederMotor() {}

    public default void zeroIndexerEncoder() {}
    public default void zeroFeederEncoder() {}
}
