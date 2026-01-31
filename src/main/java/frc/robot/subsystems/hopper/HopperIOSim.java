package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

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
    private ElevatorSim hopperSim;
    private PIDController pid;
    private double inputVoltage = 0.0;

    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d slider;
    private final LoggedMechanism2d mechanism;

    public HopperIOSim() {

        hopperSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), 
            HopperConstants.kGearing, 
            HopperConstants.HOPPER_MASS.in(Kilograms), 
            HopperConstants.HOPPER_DRUMRADIUS.in(Meters), 
            HopperConstants.HOPPER_MIN_EXTENSION.in(Meters), 
            HopperConstants.HOPPER_MAX_EXTENSION.in(Meters), 
            false, 
            0.0
            );

        pid = new PIDController(HopperConstants.SIM_PID.kP, HopperConstants.SIM_PID.kI, HopperConstants.SIM_PID.kD);
        running = false;

        mechanism = new LoggedMechanism2d(2, 2);
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
        hopperSim.setInputVoltage(0.0);
        running = false;
    }

    @Override
    public void setSetpoint(Distance setpoint) {
        pid.setSetpoint(setpoint.in(Meters));
        running = true;
    }

    public Distance getPosition() {
        return Meters.of(hopperSim.getPositionMeters());
    }

    
    public void updateInputs(HopperInputs inputs) {
        double volts = 0.0;
        double current = 0.0;
        if (running) {

            /* PID control */
            volts = MathUtil.clamp(
                pid.calculate(hopperSim.getPositionMeters()) * 12, 
                -RoboRioSim.getVInVoltage(), 
                RoboRioSim.getVInVoltage()
            );

            /* MANUAL control */
            //volts = MathUtil.clamp(manualVoltage, -RoboRioSim.getVInVoltage(), RoboRioSim.getVInVoltage());

            current = hopperSim.getCurrentDrawAmps();

        }
        hopperSim.setInputVoltage (volts);
        hopperSim.update(0.02);

        inputs.isMainMotorConnected = true;
        inputs.mainAppliedVoltage = Volts.of(volts);
        inputs.mainAppliedCurrent = Amps.of(current);
        inputs.mainMotorTemp = 0.0;
        inputs.mainMotorPosition = Meters.of(hopperSim.getPositionMeters());

        inputs.isFollowerMotorConnected = true;
        inputs.followerAppliedVoltage = Volts.of(volts);
        inputs.followerAppliedCurrent = Amps.of(current);
        inputs.followerMotorTemp = 0.0;
        inputs.followerMotorPosition = Meters.of(hopperSim.getPositionMeters());

        slider.setLength(hopperSim.getPositionMeters());
        Logger.recordOutput("Hopper Slider/Mech", mechanism);
    }



}
