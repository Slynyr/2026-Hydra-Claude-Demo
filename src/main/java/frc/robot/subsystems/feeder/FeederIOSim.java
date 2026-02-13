package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim feederSim;
    private final DCMotor motor = DCMotor.getKrakenX44(1);
    private final PIDController controller;
    private boolean running;
    private double numberOfRotations;

    public FeederIOSim() {
        feederSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                        motor,
                        0.002,
                        1
                ),
            motor
        );

        controller = new PIDController(
            FeederConstants.SIM_PID.kP, 
            FeederConstants.SIM_PID.kI,
            FeederConstants.SIM_PID.kD);
        running = false;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        feederSim.setInputVoltage(voltage);
        running = true;
    }

    @Override
    public void runRPS(double RPS) {
        controller.setSetpoint(RPS);
        running = true;
    }

    @Override
    public void stopMotor() {
        controller.setSetpoint(0.0);
        running = false;
    }

    @Override
    public AngularVelocity getVelocityRPS() {
        return RotationsPerSecond.of(feederSim.getAngularVelocityRPM()/60);
    }

    
    @Override
    public void updateInputs(FeederInputs inputs) {
        double simVoltage = 0.0;
        if (running) {
            simVoltage = MathUtil.clamp(
                    controller.calculate(getVelocityRPS().in(RotationsPerSecond)),
                    -RoboRioSim.getVInVoltage(),
                    RoboRioSim.getVInVoltage()
            );
        }
        feederSim.setInputVoltage(simVoltage);
        feederSim.update(0.02);
        
        inputs.isMotorConnected = true;
        inputs.appliedVoltage = Volts.of(feederSim.getInputVoltage());
        inputs.motorVelocity = getVelocityRPS();
        inputs.appliedCurrent = Amps.of(feederSim.getCurrentDrawAmps());
        numberOfRotations += getVelocityRPS().in(RotationsPerSecond)*0.02;
        inputs.motorPosition = Rotations.of(numberOfRotations);
    }

}
