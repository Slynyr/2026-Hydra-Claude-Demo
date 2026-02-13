package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface FeederIO {
    @AutoLog
    public class FeederInputs {
        public boolean isMotorConnected = false;
        public Voltage appliedVoltage = Volts.of(0.0);
        public Current appliedCurrent = Amps.of(0.0);
        public double motorTemperature = 0.0;
        public Angle motorPosition = Degrees.of(0.0);
        public AngularVelocity motorVelocity = RotationsPerSecond.of(0.0);
    }

    public default void updateInputs(FeederInputs inputs) {}

    public default void setMotorVoltage(double voltage) {}

     public default void runRPS(double velocity) {}

     public default void stopMotor() {}

     public default void zeroEncoder() {}
     
     public default AngularVelocity getVelocityRPS() {return RotationsPerSecond.of(0);}

     
}