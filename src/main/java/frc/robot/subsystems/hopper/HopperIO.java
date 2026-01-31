package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {
    @AutoLog
    public class HopperInputs {
        public boolean isMainMotorConnected = false;
        public Current mainAppliedCurrent = Amps.of(0.0);
        public Voltage mainAppliedVoltage = Volts.of(0.0);
        public double mainMotorTemp = 0.0;
        public Distance mainMotorPosition = Meters.of(0.0);

        public boolean isFollowerMotorConnected = false;
        public Current followerAppliedCurrent = Amps.of(0.0);
        public Voltage followerAppliedVoltage = Volts.of(0.0);
        public double followerMotorTemp = 0.0;
        public Distance followerMotorPosition = Meters.of(0.0);
    }

    public default void updateInputs(HopperInputs inputs) {}
    public default void setMotorVoltage(double voltage) {}
    public default void stopMotor() {}
    public default void zeroEncoder() {}
    public default void setSetpoint(Distance setpoint) {}
    public default Distance getPosition() {return Meters.of(0.0);}

}