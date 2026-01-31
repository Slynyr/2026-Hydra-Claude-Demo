package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs;

    public Hopper(HopperIO io) {
        this.io = io;
        inputs = new HopperInputsAutoLogged();
    }

    /** 
     * Extends hopper 0.3 metres out 
     */
    public Command fullExtend() {
        return Commands.runOnce(
            () -> io.setSetpoint(HopperConstants.HOPPER_MAX_EXTENSION), this
        );
    }

    /** 
     * Retracts hopper all the way to 0.0m
     */
    public Command fullRetract() {
        return Commands.runOnce(
            () -> io.setSetpoint(HopperConstants.HOPPER_MIN_EXTENSION), this
        );
    }

    /** 
     * Positive voltage extends, Negative voltage retracts (MAX of 0.3m and MIN of 0.0m)
     */
    public Command manualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command stopMotor() {
        return Commands.runOnce(
            () -> io.stopMotor(), this
        );
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Distance getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }
}
