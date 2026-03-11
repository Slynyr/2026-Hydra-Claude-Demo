package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX         motor;
    private final PositionVoltage positionController;

    private Distance setpoint;

    private final StatusSignal<Angle>       position;
    private final StatusSignal<Voltage>     voltage;
    private final StatusSignal<Current>     supplyCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Current>     torqueCurrent;

    public HopperIOTalonFX(int mainMotorID) {
        motor = new TalonFX(mainMotorID);
        TalonFXConfigurator config = motor.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                             .withSupplyCurrentLimit(HopperConstants.CURRENT_LIMIT)
                             .withSupplyCurrentLimitEnable(true));

        config.apply(new FeedbackConfigs().withSensorToMechanismRatio(HopperConstants.GEARING));

        config.apply(new Slot0Configs()
                             .withKP(HopperConstants.TALONFX_PID.kP)
                             .withKI(HopperConstants.TALONFX_PID.kI)
                             .withKD(HopperConstants.TALONFX_PID.kD));

        config.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        motor.setNeutralMode(NeutralModeValue.Brake);

        positionController = new PositionVoltage(0).withSlot(0);

        motor.setPosition(0);

        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();
        torqueCurrent = motor.getTorqueCurrent();

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

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public void brakeMode() {
        PhoenixUtil.tryUntilOk(3, () -> motor.setNeutralMode(NeutralModeValue.Brake));
    }

    @Override
    public void coastMode() {
        PhoenixUtil.tryUntilOk(3, () -> motor.setNeutralMode(NeutralModeValue.Coast));
    }

    @Override
    public Distance getPosition() {
        return Meters.of(position.getValueAsDouble() * HopperConstants.UNIT_CONVERSION_FACTOR);
    }

    @Override
    public void setSetpoint(Supplier<Distance> setpoint) {
        Distance setpointNew = Meters.of(MathUtil.clamp(
                setpoint.get().in(Meters) / HopperConstants.UNIT_CONVERSION_FACTOR, 0,
                HopperConstants.HOPPER_MAX_EXTENSION.in(Meters) / HopperConstants.UNIT_CONVERSION_FACTOR));
        PhoenixUtil.tryUntilOk(
                3,
                () -> motor.setControl(positionController.withPosition(setpointNew.in(Meters))
                                                         .withSlot(0)));
        this.setpoint = setpoint.get();
    }

    @Override
    public Distance getSetpoint() {
        return setpoint;
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
                position,
                voltage,
                supplyCurrent,
                temperature
        ).isOK();
        inputs.appliedVoltage = voltage.getValue();
        inputs.appliedCurrent = supplyCurrent.getValue();
        inputs.torqueCurrent = torqueCurrent.getValue();
        inputs.motorTemperature = temperature.getValueAsDouble();
        inputs.motorPosition = getPosition();
        inputs.setpoint = setpoint;
    }
}