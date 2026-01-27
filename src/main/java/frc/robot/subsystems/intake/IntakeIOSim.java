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
      
      // Ensure simulator has a battery voltage available
      RoboRioSim.setVInVoltage(12.0);

      extensionSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), 
            Extension.GEARING, 
            Extension.INTAKE_MASS.in(Kilograms), 
            Extension.INTAKE_DRUMRADIUS.in(Meters), 
            Extension.INTAKE_MIN_DISTANCE.in(Meters), 
            Extension.INTAKE_MAX_DISTANCE.in(Meters), 
            false, 
            Extension.INTAKE_MIN_DISTANCE.in(Meters)
        );

      pid = new PIDController(
            Extension.SIM_PID.kP, 
            Extension.SIM_PID.kI, 
            Extension.SIM_PID.kD
        );

      running = false;

    }

    @Override
    public void setExtensionVoltage(double voltage) {

      extensionSim.setInputVoltage(voltage);
      running = voltage != 0;

    }

    @Override
    public void setRollerVoltage(double voltage) {

      rollerVoltage = voltage;

    }

    @Override
    public void setSetpoint(Distance position) {

      pid.setSetpoint(position.in(Meters));
      running = true;

    }

    @Override
    public Distance getPosition() {

      return Meters.of(extensionSim.getPositionMeters());

    } 

    @Override
    public void updateInputs(IntakeInputs inputs) {

        double volts = 0.0;

        if (running) {
            double pidOut = pid.calculate(extensionSim.getPositionMeters());
            // use RoboRioSim voltage if available, otherwise fallback to 12V
            double maxV = Math.max(12.0, RoboRioSim.getVInVoltage());
            volts = MathUtil.clamp(pidOut * 12.0, -maxV, maxV);
        }

      extensionSim.setInputVoltage(volts);
      extensionSim.update(0.02);

      inputs.extensionPosition = extensionSim.getPositionMeters();
      inputs.isExtended = extensionSim.getPositionMeters() >= Extension.EXTENSION_MAX_DISTANCE.in(Meters) - 0.01;
      inputs.isRetracted = extensionSim.getPositionMeters() <= Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;
      inputs.extensionVelocity = MetersPerSecond.of(extensionSim.getVelocityMetersPerSecond());
      inputs.extensionCurrent = Amps.of(extensionSim.getCurrentDrawAmps());
      inputs.extensionRunning = running;
      inputs.extensionVolts = Volts.of(volts);
      inputs.extensionTemp = 25.0; // Constant temp for sim

      inputs.rollerCurrent = Amps.of(rollerVoltage / 12.0 * 20.0); // Simulated current draw
      inputs.rollerVolts = Volts.of(rollerVoltage);
      inputs.rollerTemp = 25.0;
      inputs.rollerVelocity = MetersPerSecond.of(rollerVoltage / 12.0 * 5000.0); // Simulated velocity


      inputs.isRollerConnected = true;
      inputs.isExtensionConnected = true;
    }
}