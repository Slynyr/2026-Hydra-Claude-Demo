package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class SerializerIOTalonFX implements SerializerIO {

    private final TalonFX motor;

    private final StatusSignal<AngularVelocity> indexerDeviceVelocity;
    private final StatusSignal<Angle>           indexerDevicePosition;
    private final StatusSignal<Voltage>         indexerDeviceVoltage;
    private final StatusSignal<Current>         indexerDeviceCurrent;
    private final StatusSignal<Current>         indexerDeviceCurrentStator;
    private final StatusSignal<Temperature>     indexerDeviceTemp;

    private Voltage targetVoltage = Volts.of(0);

    /**
     * @param serializerId motor can id of the serializer itself
     */
    public SerializerIOTalonFX(int serializerId) {
        motor = new TalonFX(serializerId);

        final TalonFXConfigurator upperMotorConfig = motor.getConfigurator();

        final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SerializerConstants.CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true);
        upperMotorConfig.apply(currentConfigs);

        upperMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        motor.setNeutralMode(NeutralModeValue.Brake);

        indexerDeviceVelocity = motor.getVelocity();
        indexerDevicePosition = motor.getPosition();
        indexerDeviceVoltage = motor.getMotorVoltage();
        indexerDeviceCurrent = motor.getSupplyCurrent();
        indexerDeviceCurrentStator = motor.getStatorCurrent();
        indexerDeviceTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                indexerDevicePosition,
                indexerDeviceVelocity,
                indexerDeviceVoltage,
                indexerDeviceCurrent,
                indexerDeviceTemp,
                indexerDeviceCurrentStator
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void setVoltage(double voltage) {
        targetVoltage = Volts.of(voltage);
        motor.setVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        motor.stopMotor();
    }

    @Override
    public AngularVelocity getVelocity() {
        return indexerDeviceVelocity.getValue();
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isSerializerConnected = BaseStatusSignal.refreshAll(
            indexerDevicePosition,
            indexerDeviceVelocity,
            indexerDeviceVoltage,
            indexerDeviceCurrent,
            indexerDeviceTemp,
            indexerDeviceCurrentStator
            ).isOK();
        Logger.recordOutput("Serializer/StatorCurrent", indexerDeviceCurrentStator.getValue());
        inputs.serializerPosition = indexerDevicePosition.getValue();
        inputs.serializerVelocity = indexerDeviceVelocity.getValue();
        inputs.serializerVoltage = indexerDeviceVoltage.getValue();
        inputs.serializerCurrent = indexerDeviceCurrent.getValue();
        inputs.serializerTemperature = indexerDeviceTemp.getValueAsDouble();

        inputs.targetVoltage = targetVoltage;
    }
}