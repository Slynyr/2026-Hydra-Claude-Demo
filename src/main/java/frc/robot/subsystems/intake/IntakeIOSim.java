package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.intake.IntakeConstants.Extension;

import static edu.wpi.first.units.Units.*;

public class IntakeIOSim implements IntakeIO {

    private final ElevatorSim   extension;
    private final PIDController extensionController;

    private boolean isRunning;
    private double  rollerVoltage = 0.0;

    public IntakeIOSim() {
        RoboRioSim.setVInVoltage(12.0);

        extension = new ElevatorSim(
                DCMotor.getKrakenX44(1),
                Extension.GEARING,
                Extension.INTAKE_MASS.in(Kilograms),
                0.1,
                Extension.EXTENSION_MIN_DISTANCE.in(Meters),
                Extension.EXTENSION_MAX_DISTANCE.in(Meters),
                false,
                Extension.EXTENSION_MIN_DISTANCE.in(Meters)
        );

        extensionController = new PIDController(
                Extension.SIM_PID.getP(),
                Extension.SIM_PID.getI(),
                Extension.SIM_PID.getD()
        );

        isRunning = false;
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        extension.setInputVoltage(voltage);
        isRunning = voltage != 0;
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerVoltage = voltage;
    }

    @Override
    public void setSetpoint(Distance position) {
        extensionController.setSetpoint(position.in(Meters));
        isRunning = true;
    }

    @Override
    public Distance getPosition() {
        return Meters.of(extension.getPositionMeters());
    }

    @Override
    public void stopMotor() {
        extensionController.reset();
        isRunning = false;
        rollerVoltage = 0.0;
        setExtensionVoltage(0);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        double volts = 0.0;

        if (isRunning) {
            double pidOut = extensionController.calculate(extension.getPositionMeters());
            double maxV = Math.max(Extension.MAX_VOLTAGE.in(Volts), RoboRioSim.getVInVoltage());
            volts = MathUtil.clamp(pidOut, -maxV, maxV);
        }

        extension.setInputVoltage(volts);
        extension.update(0.02);

        inputs.extensionVelocity = MetersPerSecond.of(extension.getVelocityMetersPerSecond());
        inputs.extensionCurrent = Amps.of(extension.getCurrentDrawAmps());
        inputs.extensionTorqueCurrent = Amps.of(extension.getCurrentDrawAmps() * 0.5);
        inputs.isExtensionRunning = isRunning;
        inputs.extensionVolts = Volts.of(volts);
        inputs.extensionTemp = 25.0;

        inputs.rollerCurrent = Amps.of(rollerVoltage / 12.0 * 20.0);
        inputs.rollerVolts = Volts.of(rollerVoltage);
        inputs.rollerTemp = 25.0;
        inputs.rollerVelocity = RotationsPerSecond.of(rollerVoltage / 12.0 * 5000.0);

        inputs.isRollerConnected = true;
        inputs.isExtensionConnected = true;

        extensionController.setPID(Extension.SIM_PID.getP(), Extension.SIM_PID.getI(), Extension.SIM_PID.getD());
    }
}