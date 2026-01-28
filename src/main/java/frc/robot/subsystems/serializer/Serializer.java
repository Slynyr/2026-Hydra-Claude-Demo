package frc.robot.subsystems.serializer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Serializer extends SubsystemBase{
    private SerializerIO io;
    private final SerializerInputsAutoLogged inputs;

    public Serializer(SerializerIO io) {
        this.io = io;
        inputs = new SerializerInputsAutoLogged();
    }

    public Command setFeederVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setFeederMotorVoltage(voltage);
        }, this);
    }

    public Command setIndexerVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setIndexerMotorVoltage(voltage);
        }, this);
    }

    public Command stopIndexer() {
        return Commands.runOnce(() -> {
            io.stopIndexerMotor();
        }, this);
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> {
            io.stopFeederMotor();
        }, this);
    }

    public Command zeroIndexerEncoder() {
        return Commands.runOnce(() -> {
            io.zeroIndexerEncoder();
        }, this);
    }

    public Command zeroFeederEncoder() {
        return Commands.runOnce(() -> {
            io.zeroFeederEncoder();
        }, this);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Serializer", inputs);
    }


}