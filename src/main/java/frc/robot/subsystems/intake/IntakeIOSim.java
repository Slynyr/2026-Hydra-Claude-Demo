package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.subsystems.intake.IntakeConstants.Extension;

public class IntakeIOSim implements IntakeIO {
  
    private final ElevatorSim extensionSim;
    private final PIDController pid;

    private boolean running;
    private double rollerVoltage = 0.0;

    public IntakeIOSim() {
      
      RoboRioSim.setVInVoltage(12.0);

      extensionSim = new ElevatorSim(
            DCMotor.getKrakenX44(1), 
            Extension.GEARING, 
            Extension.INTAKE_MASS.in(Kilograms), 
              0.1,
            Extension.EXTENSION_MIN_DISTANCE.in(Meters), 
            Extension.EXTENSION_MAX_DISTANCE.in(Meters), 
            false, 
            Extension.EXTENSION_MIN_DISTANCE.in(Meters)
        );

      pid = new PIDController(
        Extension.PID.getP(),
        Extension.PID.getI(),
        Extension.PID.getD()
      );

      running = false;
    }
/**
* Sets the voltage of the extension motor
* @param voltage The voltage to set the extension motor to, in volts.
 */
    @Override
    public void setExtensionVoltage(double voltage) {
      extensionSim.setInputVoltage(voltage);
      running = voltage != 0;
    }
/**
* Sets the voltage of the roller motor
* @param voltage The voltage to set the roller motor to, in volts.
 */
    @Override
    public void setRollerVoltage(double voltage) {
      rollerVoltage = voltage;
    }
/**
 * Sets the position setpoint for the extension motor
 * @param position The position to set the extension motor to, in meters.
 */
    @Override
    public void setSetpoint(Distance position) {
      pid.setSetpoint(position.in(Meters));
      running = true;
    }
/**
* Gets current motor position
* @return The current position of the intake extension, in meters.
 */
    @Override
    public Distance getPosition() {
      return Meters.of(extensionSim.getPositionMeters());
    } 
/**
* Sets motor to 0 voltage
* @return A command that stops the extension motor when executed.
 */
    @Override
    public void stopMotor() {
      pid.reset();
      running = false;
      rollerVoltage = 0.0;
      setExtensionVoltage(0);
    }
/**
 * Updates the inputs of the intake subsystem
 * @param inputs The inputs object to update with the latest sensor values and other relevant information.
 */
    @Override
    public void updateInputs(IntakeInputs inputs) {

        double volts = 0.0;

        if (running) {
            double pidOut = pid.calculate(extensionSim.getPositionMeters()); 
            double maxV = Math.max(Extension.MAX_VOLTAGE.in(Volts), RoboRioSim.getVInVoltage()); 
            volts = MathUtil.clamp(pidOut, -maxV, maxV); 
        }

      extensionSim.setInputVoltage(volts);
      extensionSim.update(0.02);

    //   inputs.extensionPosition = extensionSim.getPositionMeters();
      inputs.isExtended = extensionSim.getPositionMeters() >= Extension.EXTENSION_MAX_DISTANCE.in(Meters) - 0.01;
      inputs.isRetracted = extensionSim.getPositionMeters() <= Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;
      inputs.extensionVelocity = MetersPerSecond.of(extensionSim.getVelocityMetersPerSecond());
      inputs.extensionCurrent = Amps.of(extensionSim.getCurrentDrawAmps());
      inputs.extensionTorqueCurrent = Amps.of(extensionSim.getCurrentDrawAmps() * 0.5);
      inputs.isExtensionRunning = running;
      inputs.extensionVolts = Volts.of(volts);
      inputs.extensionTemp = 25.0;

      inputs.rollerCurrent = Amps.of(rollerVoltage / 12.0 * 20.0);
      inputs.rollerVolts = Volts.of(rollerVoltage);
      inputs.rollerTemp = 25.0;
      inputs.rollerVelocity = RotationsPerSecond.of(rollerVoltage / 12.0 * 5000.0);

      inputs.isRollerConnected = true;
      inputs.isExtensionConnected = true;

      pid.setPID(Extension.PID.getP(), Extension.PID.getI(), Extension.PID.getD()); 
    }
}