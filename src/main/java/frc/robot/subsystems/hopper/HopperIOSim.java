package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class HopperIOSim implements HopperIO {
    private boolean running;
    private final ElevatorSim hopperSim;
    private final PIDController pid;
    private double inputVoltage = 0.0;
    private Distance simSetpoint;

    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d slider;
    private final LoggedMechanism2d mechanism;

    public HopperIOSim() {

        hopperSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), 
            HopperConstants.GEARING, 
            HopperConstants.HOPPER_MASS.in(Kilograms), 
            HopperConstants.HOPPER_DRUMRADIUS.in(Meters), 
            HopperConstants.HOPPER_MIN_EXTENSION.in(Meters), 
            HopperConstants.HOPPER_MAX_EXTENSION.in(Meters)+1000, 
            false, 
            0.0
            );

        pid = new PIDController(HopperConstants.SIM_PID.kP, HopperConstants.SIM_PID.kI, HopperConstants.SIM_PID.kD);
        running = false;

        mechanism = new LoggedMechanism2d(14, 2);
        root = mechanism.getRoot("Hopper", 1, 1);
        slider = new LoggedMechanismLigament2d("Arm", 0.3, 0);
        root.append(slider);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        hopperSim.setInputVoltage(voltage);
        inputVoltage = voltage;
        running = true;
    }

    @Override
    public void stopMotor() {
        //pid.reset();
        hopperSim.setInputVoltage(0.0);
        running = false;
    }

    @Override
    public void setSetpoint(Supplier<Distance> setpoint) {
        pid.setSetpoint(setpoint.get().in(Meters));
        simSetpoint = setpoint.get();
        running = true;
    }

    @Override
    public Distance getPosition() {
        return Meters.of(hopperSim.getPositionMeters());
    }

    @Override
    public Distance getPositionIntakeZero() {
        return getPosition().plus(HopperConstants.STARTING_GAP_TO_INTAKE);
    }

    public Distance getSetpoint() {
        return simSetpoint;
    }

    
    public void updateInputs(HopperInputs inputs) {
        running = true;
        double volts = 0.0;
        double current = 0.0;
        if (running) {

            /* PID control */
            volts = MathUtil.clamp(
                pid.calculate(hopperSim.getPositionMeters()), 
                -RoboRioSim.getVInVoltage(), 
                RoboRioSim.getVInVoltage()
            );

            /* MANUAL control */
            //volts = MathUtil.clamp(manualVoltage, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage());

            current = hopperSim.getCurrentDrawAmps();

        }
        hopperSim.setInputVoltage (volts);
        hopperSim.update(0.02);

        inputs.isMotorConnected = true;
        inputs.appliedVoltage = Volts.of(volts);
        inputs.appliedCurrent = Amps.of(current);
        inputs.motorTemp = 0.0;
        inputs.motorPosition = getPosition();
        inputs.motorPositionIntakeZero = inputs.motorPosition.plus(HopperConstants.STARTING_GAP_TO_INTAKE);
        inputs.setpoint = simSetpoint;

        slider.setLength(getPosition().in(Meters));
        Logger.recordOutput("Hopper Slider/Mech", mechanism);
    }



}
