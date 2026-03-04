package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX m_mainMotor;

    private final TalonFXConfigurator m_mainMotorConfig;

    private Distance motorSetpoint;
    private PositionVoltage m_request;
    
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<Voltage> deviceVoltage;
    private final StatusSignal<Current> deviceCurrent;
    private final StatusSignal<Temperature> deviceTemp;
    private final StatusSignal<Current> torqueCurrent;

    public HopperIOTalonFX(int mainMotorID) {
        m_mainMotor = new TalonFX(mainMotorID);

        m_mainMotorConfig = m_mainMotor.getConfigurator();

        final CurrentLimitsConfigs m_currentConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(HopperConstants.CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        m_mainMotorConfig.apply(m_currentConfig);

        final FeedbackConfigs m_encoderConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(HopperConstants.GEARING);
        m_mainMotorConfig.apply(m_encoderConfigs);

        final Slot0Configs m_pidConfig = Constants.IS_TUNING
        ? new Slot0Configs()
            .withKP(HopperConstants.PID.getP())
            .withKI(HopperConstants.PID.getI())
            .withKD(HopperConstants.PID.getD())
        : new Slot0Configs()
            .withKP(HopperConstants.TALONFX_PID.kP)
            .withKI(HopperConstants.TALONFX_PID.kI)
            .withKD(HopperConstants.TALONFX_PID.kD);

        m_mainMotorConfig.apply(m_pidConfig);

        m_mainMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_mainMotor.setNeutralMode(NeutralModeValue.Brake);

        m_request = new PositionVoltage(0).withSlot(0);

        m_mainMotor.setPosition(0);
 
        motorPosition = m_mainMotor.getPosition();
        deviceVoltage = m_mainMotor.getMotorVoltage();
        deviceCurrent = m_mainMotor.getSupplyCurrent();
        deviceTemp  = m_mainMotor.getDeviceTemp();
        torqueCurrent = m_mainMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            deviceVoltage,
            deviceCurrent,
            deviceTemp
        );

        m_mainMotor.optimizeBusUtilization();

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
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setPosition(0));
    }

    @Override
    public void brakeMode() {
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setNeutralMode(NeutralModeValue.Brake));
    }

    @Override
    public void coastMode() {
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setNeutralMode(NeutralModeValue.Coast));
    }

    @Override
    public Distance getPosition() {
        return Meters.of(motorPosition.getValueAsDouble());
    }

    @Override
    public Distance getPositionIntakeZero() {
        return getPosition().plus(HopperConstants.STARTING_GAP_TO_INTAKE);
    }

    @Override
    public void setSetpoint(Distance setpoint) {
        PhoenixUtil.tryUntilOk(3, () -> m_mainMotor.setControl(m_request.withPosition(setpoint.in(Meters)).withSlot(0)));
        motorSetpoint = setpoint;
    }

    @Override
    public Distance getSetpoint() {
        return motorSetpoint;
    }

    @Override
    public void updateInputs(HopperInputs inputs) {

        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition,
            deviceVoltage, 
            deviceCurrent, 
            deviceTemp
        ).isOK();
        inputs.appliedVoltage = deviceVoltage.getValue();
        inputs.appliedCurrent = deviceCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.motorTemp = deviceTemp.getValueAsDouble();
        inputs.motorPosition = Meters.of(motorPosition.getValueAsDouble());
        inputs.motorPositionIntakeZero = inputs.motorPosition.plus(HopperConstants.STARTING_GAP_TO_INTAKE);
        inputs.setpoint = motorSetpoint;
    }

}