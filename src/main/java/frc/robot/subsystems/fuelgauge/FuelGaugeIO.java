package frc.robot.subsystems.fuelgauge;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface FuelGaugeIO {

    @AutoLog
    class FuelGaugeInputs {
        public Voltage voltage = Volts.of(0.0);

        public double fuel = 0;
    }

    default Voltage getVoltage() {return Volts.of(0.0);}

    default void updateInputs(FuelGaugeInputs inputs) {}
}