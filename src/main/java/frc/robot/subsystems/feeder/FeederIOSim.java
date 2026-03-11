package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim flywheel;
    private final PIDController controller;

    private boolean running;
    private double  revolutions;
    private double  setpoint;

    public FeederIOSim() {
        DCMotor motor = DCMotor.getKrakenX44(1);
        flywheel = new FlywheelSim(LinearSystemId.createFlywheelSystem(motor, 0.002, 1), motor);

        controller = new PIDController(
                FeederConstants.SIM_PID.kP, FeederConstants.SIM_PID.kI, FeederConstants.SIM_PID.kD);
        running = false;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        flywheel.setInputVoltage(voltage);
        running = true;
    }

    @Override
    public void runVelocity(Supplier<AngularVelocity> velocity) {
        controller.setSetpoint(velocity.get().in(RotationsPerSecond));
        setpoint = velocity.get().in(RotationsPerSecond);
        running = true;
    }

    @Override
    public void stopMotor() {
        controller.setSetpoint(0.0);
        running = false;
    }

    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(flywheel.getAngularVelocityRPM() / 60);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        double volts = running
                       ? MathUtil.clamp(controller.calculate(getVelocity().in(RotationsPerSecond)),
                                        -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage())
                       : 0.0;

        flywheel.setInputVoltage(volts);
        flywheel.update(0.02);

        inputs.isMotorConnected = true;
        inputs.voltage = Volts.of(flywheel.getInputVoltage());
        inputs.velocity = getVelocity();
        inputs.current = Amps.of(flywheel.getCurrentDrawAmps());
        revolutions += getVelocity().in(RotationsPerSecond) * 0.02;
        inputs.position = Rotations.of(revolutions);
        inputs.setpoint = RotationsPerSecond.of(setpoint);
    }
}
