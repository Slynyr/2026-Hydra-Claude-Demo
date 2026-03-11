package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX m_motor;

    private TalonFXConfigurator m_motorConfig;

    private CurrentLimitsConfigs m_currentConfig;

    private FeedbackConfigs m_encoderConfigs;
    private Slot0Configs m_pidSlowConfig;
    private Slot1Configs m_pidFastConfig;

    private PositionVoltage m_request;

    private StatusSignal<Angle> motorPosition;
    private StatusSignal<Voltage> mainMotorVoltage;
    private StatusSignal<Current> mainMotorCurrent;
    private StatusSignal<Temperature> mainMotorTemp;
    private StatusSignal<Current> mainMotorTorqueCurrent;

    private LoggedNetworkNumber fastKP = new LoggedNetworkNumber("Elevator/fast_kP", ElevatorConstants.TALONFX_FAST_PID.kP);
    private LoggedNetworkNumber fastkI = new LoggedNetworkNumber("Elevator/fast_kI", ElevatorConstants.TALONFX_FAST_PID.kI);
    private LoggedNetworkNumber fastkD = new LoggedNetworkNumber("Elevator/fast_kD",ElevatorConstants.TALONFX_FAST_PID.kD);
    private LoggedNetworkNumber kG = new LoggedNetworkNumber("Elevator/kG",ElevatorConstants.kG);
    
    public ElevatorIOTalonFX(int mainMotorID) {

        m_motor = new TalonFX(mainMotorID);

        m_motorConfig = m_motor.getConfigurator();

        m_currentConfig = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        m_motorConfig.apply(m_currentConfig);

        m_encoderConfigs = new FeedbackConfigs()
            .withSensorToMechanismRatio(ElevatorConstants.GEARING);
        m_motorConfig.apply(m_encoderConfigs);

        m_pidSlowConfig = new Slot0Configs()
            .withKP(ElevatorConstants.TALONFX_SLOW_PID.kP)
            .withKI(ElevatorConstants.TALONFX_SLOW_PID.kI)
            .withKD(ElevatorConstants.TALONFX_SLOW_PID.kD);

        m_pidFastConfig = new Slot1Configs()
            .withKP(ElevatorConstants.TALONFX_FAST_PID.kP)
            .withKI(ElevatorConstants.TALONFX_FAST_PID.kI)
            .withKD(ElevatorConstants.TALONFX_FAST_PID.kD)
            .withKG(ElevatorConstants.kG);

        m_motorConfig.apply(m_pidSlowConfig);
        m_motorConfig.apply(m_pidFastConfig);

        m_motorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        m_motor.setNeutralMode(NeutralModeValue.Brake);

        m_request = new PositionVoltage(0).withSlot(0);

        m_motor.setPosition(0);
 
        motorPosition = m_motor.getPosition();
        mainMotorVoltage = m_motor.getMotorVoltage();
        mainMotorCurrent = m_motor.getSupplyCurrent();
        mainMotorTemp  = m_motor.getDeviceTemp();
        mainMotorTorqueCurrent = m_motor.getTorqueCurrent();

        // Update all the values
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            motorPosition,
            mainMotorVoltage,
            mainMotorCurrent,
            mainMotorTemp,
            mainMotorTorqueCurrent
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
        m_motor.setVoltage(0);;
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
    public void setSetpoint(Distance setpoint,int slot) {
        PhoenixUtil.tryUntilOk(3,
        () -> m_motor.setControl(m_request.withPosition(setpoint.in(Meters))
        .withSlot(slot)));;

    }

    /**
     * updates all the inputs values
     * @param inputs inputs values 
     */
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        // m_motor.getConfigurator().apply(
        //         new Slot1Configs()
        //                 .withKP(fastKP.get())
        //                 .withKI(fastkI.get())
        //                 .withKD(fastkD.get())
        //                 .withKG(kG.get()));

        //Update all variables values for the main motors
        inputs.isMainMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition,
            mainMotorVoltage, 
            mainMotorCurrent, 
            mainMotorTemp,
            mainMotorTorqueCurrent
        ).isOK();
        inputs.mainAppliedVoltage = mainMotorVoltage.getValue();
        inputs.mainAppliedCurrent = mainMotorCurrent.getValue();
        inputs.mainMotorTemperature = mainMotorTemp.getValueAsDouble();
        inputs.mainMotorPosition = Units.Meters.of(motorPosition.getValueAsDouble());
        inputs.mainMotorTorqueCurrent = mainMotorTorqueCurrent.getValue();
        
    }
}