package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFX implements FeederIO {
    
    private TalonFX feederMotor;
    private TalonFXConfigurator feederMotorConfig;
    private CurrentLimitsConfigs currentConfigs;
    private Slot0Configs feederPidConfigs;

    private StatusSignal<AngularVelocity> feederDeviceVelocity;
    private StatusSignal<Angle> feederDevicePosition;
    private StatusSignal<Voltage> feederDeviceVoltage;
    private StatusSignal<Current> feederDeviceCurrent;
    private StatusSignal<Temperature> feederDeviceTemp;

    public FeederIOTalonFX(int feederID) {
        feederMotor = new TalonFX(feederID);
        feederMotorConfig = feederMotor.getConfigurator();

        currentConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(FeederConstants.TALON_FX_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(true);
        feederMotorConfig.apply(currentConfigs);

        feederPidConfigs = new Slot0Configs()
            .withKP(FeederConstants.TALONFX_PID.kP)
            .withKI(FeederConstants.TALONFX_PID.kI)
            .withKD(FeederConstants.TALONFX_PID.kD)
            .withKG(FeederConstants.kG)
            .withKS(FeederConstants.kS)
            .withKV(FeederConstants.kV);
        feederMotorConfig.apply(feederPidConfigs);

        feederMotorConfig.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        feederMotor.setNeutralMode(NeutralModeValue.Brake);

        feederDeviceVelocity = feederMotor.getVelocity();
        feederDevicePosition = feederMotor.getPosition();
        feederDeviceVoltage = feederMotor.getMotorVoltage();
        feederDeviceCurrent = feederMotor.getSupplyCurrent();
        feederDeviceTemp = feederMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            feederDevicePosition,
            feederDeviceVelocity,
            feederDeviceVoltage,
            feederDeviceCurrent,
            feederDeviceTemp
        );

        feederMotor.optimizeBusUtilization();
    }


    @Override
    public void setMotorVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void runRPS(double velocity) {
        VelocityVoltage velocityVoltage = new VelocityVoltage(velocity)
                                        .withSlot(0)
                                        .withFeedForward(0);
        feederMotor.setControl(velocityVoltage);
    }


    @Override
    public void stopMotor() {
        feederMotor.stopMotor();
    }

    @Override
    public void zeroEncoder() {
        feederMotor.setPosition(0);
    }

    @Override
    public AngularVelocity getVelocityRPS() {
        return feederDeviceVelocity.getValue();
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            feederDevicePosition,
            feederDeviceVelocity,
            feederDeviceVoltage,
            feederDeviceCurrent,
            feederDeviceTemp
        ).isOK();
        inputs.motorPosition = feederDevicePosition.getValue();
        inputs.motorVelocity = feederDeviceVelocity.getValue();
        inputs.appliedVoltage = feederDeviceVoltage.getValue();
        inputs.appliedCurrent = feederDeviceCurrent.getValue();
        inputs.motorTemperature = feederDeviceTemp.getValueAsDouble();
    }

}

