package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX m_mainMotor;
    private final TalonFX m_followerMotor;

    private final TalonFXConfigurator m_mainMotorConfig;
    private final TalonFXConfigurator m_followerMotorConfig;

    private CurrentLimitsConfigs m_currentConfig;
    private FeedbackConfigs m_encoderConfigs;
    private Slot0Configs m_pidConfig;

    private PositionVoltage m_request;
    
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Voltage> mainDeviceVoltage;
    private final StatusSignal<Current> mainDeviceCurrent;
    private final StatusSignal<Temperature> mainDeviceTemp;
    private final StatusSignal<Voltage> followerDeviceVoltage;
    private final StatusSignal<Current> followerDeviceCurrent;
    private final StatusSignal<Temperature> followerDeviceTemp;

    public HopperIOTalonFX(int mainMotorID, int followerMotorID) {
        m_mainMotor = new TalonFX(mainMotorID);
        m_followerMotor = new TalonFX(followerMotorID);

        m_mainMotorConfig = m_mainMotor.getConfigurator();
        m_followerMotorConfig = m_followerMotor.getConfigurator();

        m_currentConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(HopperConstants.CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        m_mainMotorConfig.apply(m_currentConfig);
        m_followerMotorConfig.apply(m_currentConfig);

        m_encoderConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(HopperConstants.kGearing);
        m_mainMotorConfig.apply(m_encoderConfigs);
        m_followerMotorConfig.apply(m_encoderConfigs);

        m_pidConfig = new Slot0Configs()
            .withKP(HopperConstants.TALONFX_PID.kP)
            .withKI(HopperConstants.TALONFX_PID.kI)
            .withKD(HopperConstants.TALONFX_PID.kD);

        m_mainMotorConfig.apply(m_pidConfig);
        m_followerMotorConfig.apply(m_pidConfig);

        m_mainMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_mainMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followerMotor.setNeutralMode(NeutralModeValue.Brake);

        m_followerMotor.setControl(new Follower(mainMotorID, MotorAlignmentValue.Opposed));

        m_request = new PositionVoltage(0).withSlot(0);

        m_mainMotor.setPosition(0);
 
        motorPosition = m_mainMotor.getPosition();
        mainDeviceVoltage = m_mainMotor.getMotorVoltage();
        mainDeviceCurrent = m_mainMotor.getSupplyCurrent();
        mainDeviceTemp  = m_mainMotor.getDeviceTemp();
        followerDeviceVoltage = m_followerMotor.getMotorVoltage();
        followerDeviceCurrent = m_followerMotor.getSupplyCurrent();
        followerDeviceTemp = m_followerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            mainDeviceVoltage,
            mainDeviceCurrent,
            mainDeviceTemp,
            followerDeviceVoltage,
            followerDeviceCurrent, 
            followerDeviceTemp
        );

        m_mainMotor.optimizeBusUtilization();
        m_followerMotor.optimizeBusUtilization();

    }


    @Override
    public void setMotorVoltage(double voltage) {
        m_mainMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        m_mainMotor.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        m_mainMotor.setPosition(0);
    }

    @Override
    public Distance getPosition() {
        return Meters.of(m_mainMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setSetpoint(Distance setpoint) {
        m_mainMotor.setControl(m_request.withPosition(setpoint.in(Meters)));
    }

    @Override
    public void updateInputs(HopperInputs inputs) {

        inputs.isMainMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition,
            mainDeviceVoltage, 
            mainDeviceCurrent, 
            mainDeviceTemp
        ).isOK();
        inputs.mainAppliedVoltage = mainDeviceVoltage.getValue();
        inputs.mainAppliedCurrent = mainDeviceCurrent.getValue();
        inputs.mainMotorTemp = mainDeviceTemp.getValueAsDouble();
        inputs.mainMotorPosition = Meters.of(motorPosition.getValueAsDouble());
        
        inputs.isFollowerMotorConnected = BaseStatusSignal.refreshAll(
            followerDeviceVoltage, 
            followerDeviceCurrent, 
            followerDeviceTemp
        ).isOK();
        inputs.followerAppliedVoltage = followerDeviceVoltage.getValue();
        inputs.followerAppliedCurrent = followerDeviceCurrent.getValue();
        inputs.followerMotorTemp = followerDeviceTemp.getValueAsDouble();        
        inputs.followerMotorPosition = Meters.of(motorPosition.getValueAsDouble());
    }

}