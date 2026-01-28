package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Volts;

public class SerializerIOSim implements SerializerIO {

    private double indexerVoltage = 0.0;
    private double feederVoltage = 0.0;

    public SerializerIOSim() {}

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerVoltage = voltage;
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederVoltage = voltage;
    }

    @Override
    public void stopIndexerMotor() {
        indexerVoltage = 0.0;
    }

    @Override
    public void stopFeederMotor() {
        feederVoltage = 0.0;
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isIndexerMotorConnected = true;
        inputs.isFeederMotorConnected = true;
        inputs.indexerAppliedVoltage = Volts.of(indexerVoltage);
        inputs.feederAppliedVoltage = Volts.of(feederVoltage);
    }



}
