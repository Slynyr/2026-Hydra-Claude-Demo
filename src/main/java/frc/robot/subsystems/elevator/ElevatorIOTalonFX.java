package frc.robot.subsystems.elevator;

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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX m_motor;

    private TalonFXConfigurator m_motorConfig;

    private CurrentLimitsConfigs m_currentConfig;

    private FeedbackConfigs m_encoderConfigs;
    private Slot0Configs m_pidConfig;

    private PositionVoltage m_request;

    private StatusSignal<Angle> motorPosition;
    private StatusSignal<Voltage> mainMotorVoltage;
    private StatusSignal<Current> mainMotorCurrent;
    private StatusSignal<Temperature> mainMotorTemp;


    public ElevatorIOTalonFX(int mainMotorID) {

        m_motor = new TalonFX(mainMotorID);

        m_motorConfig = m_motor.getConfigurator();

        m_currentConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        m_motorConfig.apply(m_currentConfig);

        m_encoderConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(1.0 / ElevatorConstants.kRotationConverter);
        m_motorConfig.apply(m_encoderConfigs);

        m_pidConfig = new Slot0Configs()
            .withKP(ElevatorConstants.TALONFX_PID.kP)
            .withKI(ElevatorConstants.TALONFX_PID.kI)
            .withKD(ElevatorConstants.TALONFX_PID.kD);

        m_motorConfig.apply(m_pidConfig);

        m_motorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_motor.setNeutralMode(NeutralModeValue.Brake);

        m_request = new PositionVoltage(0).withSlot(0);

        m_motor.setPosition(0);
 
        motorPosition = m_motor.getPosition();
        mainMotorVoltage = m_motor.getMotorVoltage();
        mainMotorCurrent = m_motor.getSupplyCurrent();
        mainMotorTemp  = m_motor.getDeviceTemp();

        // Update all the values
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            mainMotorVoltage,
            mainMotorCurrent,
            mainMotorTemp
        );

        m_motor.optimizeBusUtilization();
    }

    /**
     * set motor to assigned voltage
     * @param volts assigned voltage
     */
    @Override
    public void setMotorVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    /**
     * Stop the motor
     */
    @Override
    public void stopMotor() {
        m_motor.stopMotor();
    }

    /**
     * Zero the encoder values
     */
    @Override
    public void zeroEncoder() {
        m_motor.setPosition(0);
    }

    /**
     * return encoder values 
     * @return encoder values
     */
    @Override
    public Distance getPosition() {
        return Meters.of(m_motor.getPosition().getValueAsDouble());
    }

    /**
     * set elevator position to assigned setpoint
     * @param setpoint setpoint value
     */
    @Override
    public void setSetpoint(Distance setpoint) {
        m_motor.setControl(m_request.withPosition(setpoint.in(Meters)));
    }

    /**
     * updates all the inputs values
     * @param inputs inputs values 
     */
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        //Update all variables values for the main motors
        inputs.isMainMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition,
            mainMotorVoltage, 
            mainMotorCurrent, 
            mainMotorTemp
        ).isOK();
        inputs.mainAppliedVoltage = Units.Volts.of(mainMotorVoltage.getValueAsDouble());
        inputs.mainAppliedCurrent = Units.Amps.of(Math.abs(mainMotorCurrent.getValueAsDouble()));
        inputs.mainMotorTemperature = mainMotorTemp.getValueAsDouble();
        inputs.mainMotorPosition = Units.Meters.of(motorPosition.getValueAsDouble());
        
    }
}