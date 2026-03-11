package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX motor;

    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Angle>           position;
    private final StatusSignal<Voltage>         voltage;
    private final StatusSignal<Current>         current;
    private final StatusSignal<Temperature>     temperature;

    private Supplier<AngularVelocity> setpoint = () -> RotationsPerSecond.of(0.0);

    public FeederIOTalonFX(int feederID) {
        motor = new TalonFX(feederID);
        TalonFXConfigurator config = motor.getConfigurator();

        config.apply(new CurrentLimitsConfigs()
                             .withSupplyCurrentLimit(FeederConstants.CURRENT_LIMIT)
                             .withSupplyCurrentLimitEnable(true));

        config.apply(new FeedbackConfigs().withSensorToMechanismRatio(FeederConstants.GEARING));

        config.apply(new Slot0Configs()
                             .withKP(FeederConstants.TALONFX_PID.kP)
                             .withKI(FeederConstants.TALONFX_PID.kI)
                             .withKD(FeederConstants.TALONFX_PID.kD)
                             .withKV(FeederConstants.kV)
                             .withKS(FeederConstants.kS));

        config.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        motor.setNeutralMode(NeutralModeValue.Coast);

        velocity = motor.getVelocity();
        position = motor.getPosition();
        voltage = motor.getMotorVoltage();
        current = motor.getSupplyCurrent();
        temperature = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                position,
                velocity,
                voltage,
                current,
                temperature);

        motor.optimizeBusUtilization();
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void runVelocity(Supplier<AngularVelocity> velocity) {
        setpoint = velocity;
        VelocityVoltage velocityVoltage = new VelocityVoltage(velocity.get())
                .withSlot(0)
                .withFeedForward(0);
        motor.setControl(velocityVoltage);
    }

    @Override
    public void stopMotor() {
        motor.stopMotor();
    }

    @Override
    public AngularVelocity getVelocity() {
        return velocity.getValue();
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                current,
                temperature
        ).isOK();
        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.voltage = voltage.getValue();
        inputs.current = current.getValue();
        inputs.temperature = temperature.getValueAsDouble();
        inputs.setpoint = setpoint.get();
    }
}

