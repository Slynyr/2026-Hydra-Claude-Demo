package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;

public class SerializerIOSim implements SerializerIO {

    private double indexerVoltage = 0.0;
    private double bottomFeederVoltage = 0.0;

    public SerializerIOSim() {
        
    }

    @Override
    public void setVoltage(double voltage) {
        indexerVoltage = voltage;
        bottomFeederVoltage = voltage;
    }

    @Override
    public void stopMotors() {
        indexerVoltage = 0.0;
        bottomFeederVoltage = 0.0;
    }

    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(indexerVoltage);
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isIndexerMotorConnected = true;
        inputs.indexerAppliedVoltage = Volts.of(indexerVoltage);
        inputs.indexerMotorVelocity = getVelocity();

        inputs.isBottomFeederMotorConnected = true;
        inputs.bottomFeederAppliedVoltage = Volts.of(bottomFeederVoltage);
        inputs.bottomFeederMotorVelocity = getVelocity();
    }



}
