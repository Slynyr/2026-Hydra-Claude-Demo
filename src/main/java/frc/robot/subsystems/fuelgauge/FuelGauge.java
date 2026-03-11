package frc.robot.subsystems.fuelgauge;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class FuelGauge extends SubsystemBase {
    // TODO: get real number
    public static final double FUEL_PER_VOLT = 0;
    public static final double MIN_FUEL      = 0;

    private final FuelGaugeIO               io;
    private final FuelGaugeInputsAutoLogged inputs;

    public FuelGauge(FuelGaugeIO io) {
        this.io = io;
        inputs = new FuelGaugeInputsAutoLogged();
    }

    public double getFuel() {
        return FUEL_PER_VOLT * io.getVoltage().in(Volts) + MIN_FUEL;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FuelGauge", inputs);

        inputs.fuel = getFuel();
    }
}