package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

import static edu.wpi.first.units.Units.Meters;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX motor;

    private final PositionVoltage positionController;

    private final StatusSignal<Angle>       position;
    private final StatusSignal<Voltage>     voltage;
    private final StatusSignal<Current>     supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Current>     torqueCurrent;

    public ElevatorIOTalonFX(int id) {
        motor = new TalonFX(id);

        TalonFXConfigurator config = motor.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                             .withSupplyCurrentLimit(ElevatorConstants.CURRENT_LIMIT)
                             .withSupplyCurrentLimitEnable(true));

        config.apply(new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.GEARING));

        // default/safe (slower) PID on slot 0
        config.apply(new Slot0Configs()
                             .withKP(ElevatorConstants.TALONFX_SLOW_PID.kP)
                             .withKI(ElevatorConstants.TALONFX_SLOW_PID.kI)
                             .withKD(ElevatorConstants.TALONFX_SLOW_PID.kD));

        // more aggressive (faster) PID on slot 1
        config.apply(new Slot1Configs()
                             .withKP(ElevatorConstants.TALONFX_FAST_PID.kP)
                             .withKI(ElevatorConstants.TALONFX_FAST_PID.kI)
                             .withKD(ElevatorConstants.TALONFX_FAST_PID.kD)
                             .withKG(ElevatorConstants.kG));

        config.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        motor.setNeutralMode(NeutralModeValue.Brake);

        positionController = new PositionVoltage(0).withSlot(0);

        motor.setPosition(0);

        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();
        torqueCurrent = motor.getTorqueCurrent();

        // Update all the values
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                voltage,
                supplyCurrent,
                temperature,
                torqueCurrent
        );

        motor.optimizeBusUtilization();
    }

    /**
     * set motor to assigned voltage
     *
     * @param volts assigned voltage
     */
    @Override
    public void setMotorVoltage(double volts) {
        motor.setVoltage(volts);
    }

    /**
     * Stop the motor
     */
    @Override
    public void stopMotor() {
        motor.setVoltage(0);
    }

    /**
     * Zero the encoder values
     */
    @Override
    public void zeroEncoder() {
        motor.setPosition(0);
    }

    /**
     * return encoder values
     *
     * @return encoder values
     */
    @Override
    public Distance getPosition() {
        return Meters.of(motor.getPosition().getValueAsDouble());
    }

    /**
     * set elevator position to assigned setpoint
     *
     * @param setpoint setpoint value
     */
    @Override
    public void setSetpoint(Distance setpoint, int slot) {
        PhoenixUtil.tryUntilOk(
                3,
                () -> motor.setControl(positionController.withPosition(setpoint.in(Meters)).withSlot(slot)));
    }

    /**
     * updates all the inputs values
     *
     * @param inputs inputs values
     */
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
                position,
                voltage,
                supplyCurrent,
                temperature,
                torqueCurrent
        ).isOK();

        inputs.voltage = voltage.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.temperature = temperature.getValueAsDouble();
        inputs.position = Units.Meters.of(position.getValueAsDouble());
        inputs.torqueCurrent = torqueCurrent.getValue();
    }
}