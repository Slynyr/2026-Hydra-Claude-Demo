package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

@Deprecated
public class ElevatorIOSim implements ElevatorIO {
    private final ElevatorSim   elevator;
    private final PIDController controller;

    private boolean running;

    public ElevatorIOSim() {

        //Creating sim elevator object
        elevator = new ElevatorSim(
                DCMotor.getFalcon500(2),
                GEARING,
                ELEVATOR_MASS.in(Kilograms),
                DRUM_RADIUS.in(Inches),
                ELEVATOR_MIN_HEIGHT.in(Inches),
                ELEVATOR_MAX_HEIGHT.in(Inches),
                true,
                ELEVATOR_MIN_HEIGHT.in(Inches)
        );
        controller = new PIDController(SIM_PID.kP, SIM_PID.kI, SIM_PID.kD);
        running = false;
    }

    /**
     * Set voltage of the motor to assigned voltage
     *
     * @param voltage voltage value
     */
    @Override
    public void setMotorVoltage(double voltage) {
        elevator.setInputVoltage(voltage);
    }

    /**
     * Stops the motor
     */
    @Override
    public void stopMotor() {
        elevator.setInputVoltage(0.0);
        running = false;
    }

    /**
     * sets the elevator to the assigned setpoint
     *
     * @param setpoint setpoint value
     */
    @Override
    public void setSetpoint(Distance setpoint, int slot) {
        controller.setSetpoint(setpoint.in(Meters));
        running = true;
    }

    /**
     * returns the elevator encoders value
     *
     * @return values of the encoder
     */
    @Override
    public Distance getPosition() {
        return Meters.of(elevator.getPositionMeters());
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {
            volts = MathUtil.clamp(
                    controller.calculate(elevator.getPositionMeters()) * 12,
                    -RoboRioSim.getVInVoltage(),
                    RoboRioSim.getVInVoltage()
            );
            current = elevator.getCurrentDrawAmps();
        }
        elevator.setInputVoltage(volts);
        elevator.update(0.02);

        inputs.isConnected = true;
        inputs.voltage = Units.Volts.of(volts);
        inputs.supplyCurrent = Units.Amps.of(Math.abs(current));
        inputs.temperature = 0.0;
        inputs.position = Units.Meters.of(elevator.getPositionMeters());
    }
}