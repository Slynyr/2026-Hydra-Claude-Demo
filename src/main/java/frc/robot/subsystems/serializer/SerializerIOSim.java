package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;

public class SerializerIOSim implements SerializerIO {

    private double indexerVoltage = 0.0;

    public SerializerIOSim() {
        
    }

    @Override
    public void setMotorVoltage(double voltage) {

        indexerVoltage = voltage;

    }

    @Override
    public void stopMotor() {

        indexerVoltage = 0.0;

    }

    @Override
    public AngularVelocity getVelocity() {

        return RotationsPerSecond.of(indexerVoltage);

    }

    @Override
    public void updateInputs(SerializerInputs inputs) {

        inputs.isMotorConnected = true;
        inputs.appliedVoltage = Volts.of(indexerVoltage);
        inputs.motorVelocity = getVelocity();

    }



}
