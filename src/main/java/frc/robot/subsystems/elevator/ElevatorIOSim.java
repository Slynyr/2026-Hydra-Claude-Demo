package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.Units;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO {

    private boolean running;
    private ElevatorSim elevatorSim;
    private PIDController PID;

    public ElevatorIOSim() {

        //Creating sim elevator object
        elevatorSim = new ElevatorSim(
            DCMotor.getFalcon500(2), 
            ElevatorConstants.GEARING, 
            ElevatorConstants.ELEVATOR_MASS.in(Kilograms), 
            ElevatorConstants.ELEVATOR_DRUMRADIUS.in(Meters), 
            ElevatorConstants.ELEVATOR_MIN_HEIGHT, 
            ElevatorConstants.ELEVATOR_MAX_HEIGHT, 
            true, 
            ElevatorConstants.ELEVATOR_MIN_HEIGHT
        );
        PID = new PIDController(ElevatorConstants.SIM_PID.kP, ElevatorConstants.SIM_PID.kI, ElevatorConstants.SIM_PID.kD);
        running = false;

    }

    /**
     * Set voltage of the motor to assigned voltage 
     *@param voltage voltage value
     */
    @Override
    public void setMotorVoltage(double voltage) {
        elevatorSim.setInputVoltage(voltage);
    }

    /**
     * Stops the motor
     */
    @Override
    public void stopMotor() {
        elevatorSim.setInputVoltage(0.0);
        running = false;
    }

    /**
     * sets the elevator to the assigned setpoint
     * @param setpoint setpoint value
     */
    @Override
    public void setSetpoint(Distance setpoint) {
        PID.setSetpoint(setpoint.in(Meters));
        running = true;
    }

    /**
     * returns the elevator encoders value
     * @return values of the encoder
     */
    @Override
    public Distance getPosition() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {
            volts = MathUtil.clamp(
                PID.calculate(elevatorSim.getPositionMeters()) * 12, 
                -RoboRioSim.getVInVoltage(), 
                RoboRioSim.getVInVoltage()
            );
            current = elevatorSim.getCurrentDrawAmps();
        }
        elevatorSim.setInputVoltage (volts);
        elevatorSim.update(0.02);

        inputs.isMainMotorConnected = true;
        inputs.mainAppliedVoltage = Units.Volts.of(volts);
        inputs.mainAppliedCurrent = Units.Amps.of(Math.abs(current));
        inputs.mainMotorTemperature = 0.0;
        inputs.mainMotorPosition = Units.Meters.of(elevatorSim.getPositionMeters());
    }
}