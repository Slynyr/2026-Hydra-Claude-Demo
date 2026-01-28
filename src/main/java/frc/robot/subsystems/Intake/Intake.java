package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeInputsAutoLogged;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged inputs;
    
    public Intake(IntakeIO intakeIO) {

        this.intakeIO = intakeIO;
        inputs = new IntakeInputsAutoLogged();

    }

    public Command intakeCommand(double voltage) {

        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage), this);

    }

    public Command brakemodeCommand() {

        return Commands.runOnce(() -> intakeIO.brakeMode(), this);

    }

    public Command extendCommand() {

         return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(0.5)), this);

    }

    public Command retractCommand() {

        return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(0.0)), this);

    }

    @Override
    public void periodic() {

        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

    }
}