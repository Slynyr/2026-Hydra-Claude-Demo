package frc.robot.subsystems.intake;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.subsystems.launcher.LauncherConstants;

public final class IntakeIOTalonFX implements IntakeIO {
  
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private Distance setpoint = Meters.of(0.0);

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



        TalonFXConfiguration extensionConfigurator = new TalonFXConfiguration()
        .withFeedback(
            new FeedbackConfigs()
            .withSensorToMechanismRatio(Extension.GEARING)
        );


        extensionConfigurator.Slot0 = new Slot0Configs()
            .withKP(Extension.TALONFX_PID.kP)
            .withKI(Extension.TALONFX_PID.kI)
            .withKD(Extension.TALONFX_PID.kD);

        extensionConfigurator.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        extensionConfigurator.withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
            );
        extensionMotor.getConfigurator().apply(extensionConfigurator);

        rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

        rollerMotor.getConfigurator().apply( new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
            );


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

        extensionMotor.setPosition(0);

        rollerMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();

    }
/**
* Sets the voltage of the roller motor
* @param voltage The voltage to set the roller motor to, in volts.
 */
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }
/**
* Sets the voltage of the extension motor
* @param voltage The voltage to set the extension motor to, in volts.
 */
    public void setExtensionVoltage(double voltage) {
        extensionMotor.setVoltage(voltage);
    }
/**
* Moves intake to given setpoint
* @param position The position to set the extension motor to given position, in meters.
 */
    public void setSetpoint(Distance position) {
        setpoint = position;
        Logger.recordOutput("Intake/Setpoint", position);
        position = Meters.of(MathUtil.clamp(position.in(Meters)/Extension.UNIT_CONVERSION_FACTOR, 0, Extension.EXTENSION_MAX_DISTANCE.in(Meters)/Extension.UNIT_CONVERSION_FACTOR));
        extensionMotor.setControl(positionControl.withPosition(position.in(Meters)).withSlot(0));
    }
/**
* Sets motor to coastMode
* @return A command that sets the extension motor to coast mode when executed.
 */
    public void coastMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        extensionMotor.setNeutralMode(NeutralModeValue.Coast);
    }
/**
* Sets motor to brakeMode
* @return A command that sets the extension motor to brake mode when executed.
 */
    public void brakeMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    }   
/**
* Stops the motor
* @return A command that stops the extension motor when executed.
 */
    public void stopMotor() {
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);
    }
/**
* Gets current motor position
* @return The current position of the intake extension, in meters.
 */
    public Distance getPosition() {
        return Meters.of(extensionPositionSignal.getValueAsDouble() * Extension.UNIT_CONVERSION_FACTOR);
    }
/**
* Updates the inputs of the intake subsystem
* @param inputs The inputs object to update with the latest sensor values and other relevant information.
 */
    @Override
    public void updateInputs(IntakeIO.IntakeInputs inputs) {
        inputs.isExtensionConnected = BaseStatusSignal.refreshAll(
                extensionPositionSignal,
                extensionTemperatureSignal,
                extensionVoltageSignal,
                extensionCurrentSignal,
                extensionVelocitySignal
        ).isOK();

        inputs.extensionVolts = extensionVoltageSignal.getValue();

        inputs.extensionCurrent = Amps.of(extensionCurrentSignal.getValueAsDouble());
        inputs.extensionTorqueCurrent = Amps.of(extensionTorqueCurrentSignal.getValueAsDouble());
        inputs.extensionTemp = extensionTemperatureSignal.getValueAsDouble();

        inputs.extensionPosition = getPosition();

        inputs.extensionVelocity = MetersPerSecond.of(extensionVelocitySignal.getValueAsDouble());
        inputs.isExtensionRunning = Math.abs(extensionVoltageSignal.getValueAsDouble()) > 0.1;
        // inputs.isExtended = inputs.extensionPosition.gte(Extension.EXTENSION_MAX_DISTANCE).minus(Meters.of(0.01));
        // inputs.isRetracted = inputs.extensionPosition <= Extension.EXTENSION_MIN_DISTANCE.in(Meters) + 0.01;

        inputs.isRollerConnected = BaseStatusSignal.refreshAll(
                rollerPositionSignal,
                rollerTemperatureSignal,
                rollerVoltageSignal,
                rollerCurrentSignal,
                rollerVelocitySignal
        ).isOK();

        inputs.rollerVolts = Volts.of(rollerVoltageSignal.getValueAsDouble());
        inputs.rollerCurrent = Amps.of(rollerCurrentSignal.getValueAsDouble());
        inputs.rollerTemp = rollerTemperatureSignal.getValueAsDouble();
        inputs.rollerVelocity = RotationsPerSecond.of(rollerVelocitySignal.getValueAsDouble());
        inputs.extensionSetpoint = setpoint;

    }
}