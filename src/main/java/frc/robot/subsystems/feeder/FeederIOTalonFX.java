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
    private final TalonFX upperMotor;
    private final TalonFX lowerMotor;

    private final StatusSignal<AngularVelocity> velocityUpper;
    private final StatusSignal<Angle>           positionUpper;
    private final StatusSignal<Voltage>         voltageUpper;
    private final StatusSignal<Current>         currentUpper;
    private final StatusSignal<Temperature>     temperatureUpper;

    private final StatusSignal<AngularVelocity> velocityLower;
    private final StatusSignal<Angle>           positionLower;
    private final StatusSignal<Voltage>         voltageLower;
    private final StatusSignal<Current>         currentLower;
    private final StatusSignal<Temperature>     temperatureLower;

    private Supplier<AngularVelocity> upperSetpoint = () -> RotationsPerSecond.of(0.0);
    private Supplier<AngularVelocity> lowerSetpoint = () -> RotationsPerSecond.of(0.0);

    public FeederIOTalonFX(int feederId, int lowerFeederId) {
        upperMotor = new TalonFX(feederId);
        lowerMotor = new TalonFX(lowerFeederId);

        TalonFXConfigurator upperConfig = upperMotor.getConfigurator();
        TalonFXConfigurator lowerConfig = lowerMotor.getConfigurator();

        var currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(FeederConstants.CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);

        upperConfig.apply(currentLimits);
        lowerConfig.apply(currentLimits);

        var gearing = new FeedbackConfigs().withSensorToMechanismRatio(FeederConstants.GEARING);
        upperConfig.apply(gearing);
        lowerConfig.apply(gearing);

        upperConfig.apply(new Slot0Configs()
                             .withKP(FeederConstants.TALONFX_PID_UPPER.kP)
                             .withKI(FeederConstants.TALONFX_PID_UPPER.kI)
                             .withKD(FeederConstants.TALONFX_PID_UPPER.kD)
                             .withKV(FeederConstants.kV_UPPER)
                             .withKS(FeederConstants.kS_UPPER));
        
        lowerConfig.apply(new Slot0Configs()
                             .withKP(FeederConstants.TALONFX_PID_LOWER.kP)
                             .withKI(FeederConstants.TALONFX_PID_LOWER.kI)
                             .withKD(FeederConstants.TALONFX_PID_LOWER.kD)
                             .withKV(FeederConstants.kV_LOWER)
                             .withKS(FeederConstants.kS_LOWER));

        upperConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        lowerConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

        upperMotor.setNeutralMode(NeutralModeValue.Coast);
        lowerMotor.setNeutralMode(NeutralModeValue.Coast);

        velocityUpper = upperMotor.getVelocity();
        positionUpper = upperMotor.getPosition();
        voltageUpper = upperMotor.getMotorVoltage();
        currentUpper = upperMotor.getSupplyCurrent();
        temperatureUpper = upperMotor.getDeviceTemp();

        velocityLower = lowerMotor.getVelocity();
        positionLower = lowerMotor.getPosition();
        voltageLower = lowerMotor.getMotorVoltage();
        currentLower = lowerMotor.getSupplyCurrent();
        temperatureLower = lowerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,

                positionUpper,
                velocityUpper,
                voltageUpper,
                currentUpper,
                temperatureUpper,

                positionLower,
                velocityLower,
                voltageLower,
                currentLower,
                temperatureLower
        );

        upperMotor.optimizeBusUtilization();
        lowerMotor.optimizeBusUtilization();
    }

    @Override
    public void setVoltage(double voltage) {
        upperMotor.setVoltage(voltage);
    }

    @Override
    public void setUpperFeederVelocity(Supplier<AngularVelocity> velocity) {
        upperSetpoint = velocity;
        upperMotor.setControl(new VelocityVoltage(velocity.get())
                                      .withSlot(0)
                                      .withFeedForward(0));
    }

    @Override
    public void setLowerFeederVelocity(Supplier<AngularVelocity> velocity) {
        lowerSetpoint = () -> velocity.get();
        lowerMotor.setControl(new VelocityVoltage(lowerSetpoint.get())
                                      .withSlot(0)
                                      .withFeedForward(0));
    }

    @Override
    public void stopMotors() {
        upperMotor.stopMotor();
        lowerMotor.stopMotor();
    }

    @Override
    public AngularVelocity getUpperFeederVelocity() {
        return velocityUpper.getValue();
    }

    @Override
    public AngularVelocity getLowerFeederVelocity() {
        return velocityLower.getValue();
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.isUpperMotorConnected = BaseStatusSignal.refreshAll(
                positionUpper,
                velocityUpper,
                voltageUpper,
                currentUpper,
                temperatureUpper
        ).isOK();
        inputs.upperPosition = positionUpper.getValue();
        inputs.upperVelocity = velocityUpper.getValue();
        inputs.upperVoltage = voltageUpper.getValue();
        inputs.upperCurrent = currentUpper.getValue();
        inputs.upperTemperature = temperatureUpper.getValueAsDouble();
        inputs.upperSetpoint = upperSetpoint.get();

        inputs.isLowerMotorConnected = BaseStatusSignal.refreshAll(
                positionLower,
                velocityLower,
                voltageLower,
                currentLower,
                temperatureLower
        ).isOK();
        inputs.lowerPosition = positionLower.getValue();
        inputs.lowerVelocity = velocityLower.getValue();
        inputs.lowerVoltage = voltageLower.getValue();
        inputs.lowerCurrent = currentLower.getValue();
        inputs.lowerTemperature = temperatureLower.getValueAsDouble();
        inputs.lowerSetpoint = lowerSetpoint.get();
    }
}

