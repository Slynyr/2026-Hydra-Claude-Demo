package frc.robot.subsystems.fuelgauge;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.subsystems.fuelgauge.FuelGaugeIO.FuelGaugeInputs;

public class FuelGaugeIOAnalogInput implements FuelGaugeIO {
    private final AnalogInput ultrasonic;

    public FuelGaugeIOAnalogInput (int channel) {
        ultrasonic = new AnalogInput(channel);
    }

    @Override
    public Voltage getVoltage() {
        return Volts.of(ultrasonic.getValue());
    }

    @Override
    public void updateInputs(FuelGaugeInputs inputs) {
        inputs.voltage = getVoltage();
    }
}
