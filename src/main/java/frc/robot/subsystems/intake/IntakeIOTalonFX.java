package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.IntakeConstants.Extension;;

public final class IntakeIOTalonFX implements IntakeIO {
  
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private final PositionVoltage positionControl;

    private final StatusSignal<Angle>       rollerPositionSignal;
    private final StatusSignal<Temperature> rollerTemperatureSignal;
    private final StatusSignal<Voltage>     rollerVoltageSignal;
    private final StatusSignal<Current>     rollerCurrentSignal;
    private final StatusSignal<AngularVelocity>       rollerVelocitySignal;

    private final StatusSignal<Angle>       extensionPositionSignal;
    private final StatusSignal<Temperature> extensionTemperatureSignal;
    private final StatusSignal<Voltage>     extensionVoltageSignal;
    private final StatusSignal<Current>     extensionCurrentSignal;
    private final StatusSignal<AngularVelocity>       extensionVelocitySignal;
    private final StatusSignal<Current>    extensionTorqueCurrentSignal;

    public IntakeIOTalonFX(int rollerMotorId, int extensionMotorId) {

        rollerMotor = new TalonFX(rollerMotorId);
        extensionMotor = new TalonFX(extensionMotorId);
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);

        positionControl = new PositionVoltage(0.0);
        positionControl.withSlot(0);

        TalonFXConfiguration extensionConfigurator = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Extension.GEARING));

        if (frc.robot.Constants.IS_TUNING) {
            if (Extension.INTAKE_IS_TUNING) {
                extensionConfigurator.Slot0 = new Slot0Configs()
                    .withKP(Extension.PID.getP())
                    .withKI(Extension.PID.getI())
                    .withKD(Extension.PID.getD());
            } else {
                extensionConfigurator.Slot0 = new Slot0Configs()
                    .withKP(Extension.TALONFX_PID.kP)
                    .withKI(Extension.TALONFX_PID.kI)
                    .withKD(Extension.TALONFX_PID.kD);
            }
        } else{
            extensionConfigurator.Slot0 = new Slot0Configs()
                .withKP(Extension.TALONFX_PID.kP)
                .withKI(Extension.TALONFX_PID.kI)
                .withKD(Extension.TALONFX_PID.kD);
        }
        extensionMotor.getConfigurator().apply(extensionConfigurator);

        extensionPositionSignal    = extensionMotor.getPosition();
        extensionTemperatureSignal = extensionMotor.getDeviceTemp();
        extensionVoltageSignal     = extensionMotor.getMotorVoltage();
        extensionCurrentSignal     = extensionMotor.getSupplyCurrent();
        extensionVelocitySignal    = extensionMotor.getVelocity();
        extensionTorqueCurrentSignal = extensionMotor.getTorqueCurrent();

        rollerPositionSignal    = rollerMotor.getPosition();
        rollerTemperatureSignal = rollerMotor.getDeviceTemp();
        rollerVoltageSignal     = rollerMotor.getMotorVoltage();
        rollerCurrentSignal     = rollerMotor.getSupplyCurrent();
        rollerVelocitySignal    = rollerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(

            Extension.UPDATE_FREQUENCY,   
            extensionPositionSignal,
            extensionTemperatureSignal,
            extensionVoltageSignal,
            extensionCurrentSignal,

            rollerPositionSignal,
            rollerTemperatureSignal,
            rollerVoltageSignal,
            rollerCurrentSignal
        );

        rollerMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();

    }

    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    public void setExtensionVoltage(double voltage) {
        extensionMotor.setVoltage(voltage);
    }

    public void setSetpoint(Distance position) {
        extensionMotor.setControl(positionControl.withPosition(position.in(Meters)).withSlot(0));
    }

    public void coastMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        extensionMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brakeMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    }   

    public void stopMotor() {
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);
    }

    public Distance getPosition() {
        return Meters.of(extensionPositionSignal.getValueAsDouble());
    }

    @Override
    public void updateInputs(IntakeIO.IntakeInputs inputs) {

        inputs.isExtensionConnected = true;
        inputs.extensionVolts = Volts.of(extensionVoltageSignal.getValueAsDouble());
        inputs.extensionCurrent = Amps.of(extensionCurrentSignal.getValueAsDouble());
        inputs.extensionTorqueCurrent = Amps.of(extensionTorqueCurrentSignal.getValueAsDouble());
        inputs.extensionTemp = extensionTemperatureSignal.getValueAsDouble();
        inputs.extensionPosition =  Units.rotationsToRadians(extensionPositionSignal.getValueAsDouble());
        inputs.extensionVelocity = MetersPerSecond.of(extensionVelocitySignal.getValueAsDouble());
        inputs.isExtensionRunning = Math.abs(extensionVoltageSignal.getValueAsDouble()) > 0.1;
        inputs.isExtended = inputs.extensionPosition >= Extension.EXTENSION_MAX_DISTANCE.in(Meters) - 0.01;
        inputs.isRetracted = inputs.extensionPosition <= Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;

        inputs.isRollerConnected = true;
        inputs.rollerVolts = Volts.of(rollerVoltageSignal.getValueAsDouble());
        inputs.rollerCurrent = Amps.of(rollerCurrentSignal.getValueAsDouble());
        inputs.rollerTemp = rollerTemperatureSignal.getValueAsDouble();
        inputs.rollerVelocity = RotationsPerSecond.of(rollerVelocitySignal.getValueAsDouble());

    }
}