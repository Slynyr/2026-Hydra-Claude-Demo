package frc.robot.subsystems.serializer;

import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class SerializerIOSim implements SerializerIO {
    private double indexerVoltage      = 0.0;

    public SerializerIOSim() {
    }

    @Override
    public void setVoltage(double voltage) {
        indexerVoltage = voltage;
    }

    @Override
    public void stopMotors() {
        indexerVoltage = 0.0;
    }

    @Override
    public AngularVelocity getVelocity() {
        return RotationsPerSecond.of(indexerVoltage);
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isSerializerConnected = true;
        inputs.serializerVoltage = Volts.of(indexerVoltage);
        inputs.serializerVelocity = getVelocity();
    }
}
