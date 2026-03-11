package frc.robot.subsystems.fuelgauge;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;

import static edu.wpi.first.units.Units.Volts;

public class FuelGaugeIOAnalogInput implements FuelGaugeIO {
    private final AnalogInput ultrasonic;

    public FuelGaugeIOAnalogInput(int channel) {
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
