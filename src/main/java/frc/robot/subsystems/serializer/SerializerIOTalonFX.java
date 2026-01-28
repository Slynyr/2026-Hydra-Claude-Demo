package frc.robot.subsystems.serializer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SerializerIOTalonFX implements SerializerIO {
    
    private TalonFX indexerMotor;
    private TalonFX feederMotor;

    private TalonFXConfigurator indexerMotorConfig;
    private TalonFXConfigurator feederMotorConfig;
    private CurrentLimitsConfigs currentConfigs;

    private StatusSignal<AngularVelocity> indexerDeviceVelocity;
    private StatusSignal<Angle> indexerDevicePosition;
    private StatusSignal<Voltage> indexerDeviceVoltage;
    private StatusSignal<Current> indexerDeviceCurrent;
    private StatusSignal<Temperature> indexerDeviceTemp;

    private StatusSignal<AngularVelocity> feederDeviceVelocity;
    private StatusSignal<Angle> feederDevicePosition;
    private StatusSignal<Voltage> feederDeviceVoltage;
    private StatusSignal<Current> feederDeviceCurrent;
    private StatusSignal<Temperature> feederDeviceTemp;


    public SerializerIOTalonFX(int indexerID, int feederID) {
        indexerMotor = new TalonFX(indexerID);
        feederMotor = new TalonFX(feederID);

        indexerMotorConfig = indexerMotor.getConfigurator();
        feederMotorConfig = feederMotor.getConfigurator();
        currentConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SerializerConstants.TALON_FX_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        indexerMotorConfig.apply(currentConfigs);
        feederMotorConfig.apply(currentConfigs);

        indexerMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        feederMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        indexerMotor.setNeutralMode(NeutralModeValue.Brake);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);

        indexerDeviceVelocity = indexerMotor.getVelocity();
        indexerDevicePosition = indexerMotor.getPosition();
        indexerDeviceVoltage = indexerMotor.getMotorVoltage();
        indexerDeviceCurrent = indexerMotor.getSupplyCurrent();
        indexerDeviceTemp = indexerMotor.getDeviceTemp();

        feederDeviceVelocity = feederMotor.getVelocity();
        feederDevicePosition = feederMotor.getPosition();
        feederDeviceVoltage = feederMotor.getMotorVoltage();
        feederDeviceCurrent = feederMotor.getSupplyCurrent();
        feederDeviceTemp = feederMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            indexerDevicePosition,
            indexerDeviceVelocity,
            indexerDeviceVoltage,
            indexerDeviceCurrent,
            indexerDeviceTemp,
            
            feederDevicePosition,
            feederDeviceVelocity,
            feederDeviceVoltage,
            feederDeviceCurrent,
            feederDeviceTemp
        );

        indexerMotor.optimizeBusUtilization();
        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void stopIndexerMotor() {
        indexerMotor.stopMotor();
    }

    @Override
    public void stopFeederMotor() {
        feederMotor.stopMotor();
    }

    @Override
    public void zeroIndexerEncoder() {
        indexerMotor.setPosition(0);
    }

    @Override
    public void zeroFeederEncoder() {
        feederMotor.setPosition(0);
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isIndexerMotorConnected = BaseStatusSignal.refreshAll(
            indexerDevicePosition,
            indexerDeviceVelocity,
            indexerDeviceVoltage,
            indexerDeviceCurrent,
            indexerDeviceTemp
        ).isOK();
        inputs.indexerMotorPosition = indexerDevicePosition.getValue();
        inputs.indexerMotorVelocity = indexerDeviceVelocity.getValue();
        inputs.indexerAppliedVoltage = indexerDeviceVoltage.getValue();
        inputs.indexerAppliedCurrent = indexerDeviceCurrent.getValue();
        inputs.indexerMotorTemperature = indexerDeviceTemp.getValueAsDouble();

        
        inputs.isFeederMotorConnected = BaseStatusSignal.refreshAll(
            feederDevicePosition,
            feederDeviceVelocity,
            feederDeviceVoltage,
            feederDeviceCurrent,
            feederDeviceTemp
        ).isOK();
        inputs.feederMotorPosition = feederDevicePosition.getValue();
        inputs.feederMotorVelocity = feederDeviceVelocity.getValue();
        inputs.feederAppliedVoltage = feederDeviceVoltage.getValue();
        inputs.feederAppliedCurrent = feederDeviceCurrent.getValue();
        inputs.feederMotorTemperature = feederDeviceTemp.getValueAsDouble();
    }

}