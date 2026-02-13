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
    
    private TalonFX m_motor;

    private TalonFXConfigurator motorConfig;
    private CurrentLimitsConfigs currentConfigs;

    private StatusSignal<AngularVelocity> deviceVelocity;
    private StatusSignal<Angle> devicePosition;
    private StatusSignal<Voltage> deviceVoltage;
    private StatusSignal<Current> deviceCurrent;
    private StatusSignal<Temperature> deviceTemp;

    


    public SerializerIOTalonFX(int motorId) {
        m_motor = new TalonFX(motorId);

        motorConfig = m_motor.getConfigurator();

        currentConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(SerializerConstants.TALON_FX_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        motorConfig.apply(currentConfigs);

        motorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_motor.setNeutralMode(NeutralModeValue.Brake);

        deviceVelocity = m_motor.getVelocity();
        devicePosition = m_motor.getPosition();
        deviceVoltage = m_motor.getMotorVoltage();
        deviceCurrent = m_motor.getSupplyCurrent();
        deviceTemp = m_motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            devicePosition,
            deviceVelocity,
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        );

        m_motor.optimizeBusUtilization();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        m_motor.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        m_motor.setPosition(0);
    }

    @Override
    public AngularVelocity getVelocity() {
        return deviceVelocity.getValue();
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            devicePosition,
            deviceVelocity,
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        ).isOK();
        inputs.motorPosition = devicePosition.getValue();
        inputs.motorVelocity = deviceVelocity.getValue();
        inputs.appliedVoltage = deviceVoltage.getValue();
        inputs.appliedCurrent = deviceCurrent.getValue();
        inputs.motorTemperature = deviceTemp.getValueAsDouble();
    }

}