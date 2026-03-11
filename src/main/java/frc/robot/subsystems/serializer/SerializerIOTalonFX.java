package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class SerializerIOTalonFX implements SerializerIO {

    private final TalonFX motor;

    private final StatusSignal<AngularVelocity> indexerDeviceVelocity;
    private final StatusSignal<Angle>           indexerDevicePosition;
    private final StatusSignal<Voltage>         indexerDeviceVoltage;
    private final StatusSignal<Current>         indexerDeviceCurrent;
    private final StatusSignal<Temperature>     indexerDeviceTemp;

    private final StatusSignal<AngularVelocity> bottomFeederDeviceVelocity;
    private final StatusSignal<Angle>           bottomFeederDevicePosition;
    private final StatusSignal<Voltage>         bottomFeederDeviceVoltage;
    private final StatusSignal<Current>         bottomFeederDeviceCurrent;
    private final StatusSignal<Temperature>     bottomFeederDeviceTemp;

    /**
     * @param serializerId motor can id of the serializer itself
     * @param lowerFeederId motor can id nearest to the serializer
     */
    public SerializerIOTalonFX(int serializerId, int lowerFeederId) {
        motor = new TalonFX(serializerId);
        TalonFX lowerMotor = new TalonFX(lowerFeederId);

        final TalonFXConfigurator upperMotorConfig = motor.getConfigurator();
        final TalonFXConfigurator lowerMotorConfig = lowerMotor.getConfigurator();

        final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(SerializerConstants.CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        upperMotorConfig.apply(currentConfigs);
        lowerMotorConfig.apply(currentConfigs);

        final FeedbackConfigs encoderConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(SerializerConstants.GEARING);
        lowerMotorConfig.apply(encoderConfigs);

        upperMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        motor.setNeutralMode(NeutralModeValue.Brake);
        lowerMotor.setNeutralMode(NeutralModeValue.Brake);

        lowerMotor.setControl(new Follower(serializerId, MotorAlignmentValue.Aligned));

        indexerDeviceVelocity = motor.getVelocity();
        indexerDevicePosition = motor.getPosition();
        indexerDeviceVoltage = motor.getMotorVoltage();
        indexerDeviceCurrent = motor.getSupplyCurrent();
        indexerDeviceTemp = motor.getDeviceTemp();

        bottomFeederDeviceVelocity = lowerMotor.getVelocity();
        bottomFeederDevicePosition = lowerMotor.getPosition();
        bottomFeederDeviceVoltage = lowerMotor.getMotorVoltage();
        bottomFeederDeviceCurrent = lowerMotor.getSupplyCurrent();
        bottomFeederDeviceTemp = lowerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                indexerDevicePosition,
                indexerDeviceVelocity,
                indexerDeviceVoltage,
                indexerDeviceCurrent,
                indexerDeviceTemp,

                bottomFeederDevicePosition,
                bottomFeederDeviceVelocity,
                bottomFeederDeviceVoltage,
                bottomFeederDeviceCurrent,
                bottomFeederDeviceTemp
        );

        motor.optimizeBusUtilization();
        lowerMotor.optimizeBusUtilization();
    }

    @Override
    public void setVoltage(double voltage) {
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
                indexerDeviceTemp
        ).isOK();
        inputs.serializerPosition = indexerDevicePosition.getValue();
        inputs.serializerVelocity = indexerDeviceVelocity.getValue();
        inputs.serializerVoltage = indexerDeviceVoltage.getValue();
        inputs.serializerCurrent = indexerDeviceCurrent.getValue();
        inputs.serializerTemperature = indexerDeviceTemp.getValueAsDouble();

        inputs.isLowerFeederConnected = BaseStatusSignal.refreshAll(
                bottomFeederDevicePosition,
                bottomFeederDeviceVelocity,
                bottomFeederDeviceVoltage,
                bottomFeederDeviceCurrent,
                bottomFeederDeviceTemp
        ).isOK();
        inputs.lowerFeederPosition = bottomFeederDevicePosition.getValue();
        inputs.lowerFeederVelocity = bottomFeederDeviceVelocity.getValue();
        inputs.lowerFeederVoltage = bottomFeederDeviceVoltage.getValue();
        inputs.lowerFeederCurrent = bottomFeederDeviceCurrent.getValue();
        inputs.lowerFeederTemperature = bottomFeederDeviceTemp.getValueAsDouble();
    }
}