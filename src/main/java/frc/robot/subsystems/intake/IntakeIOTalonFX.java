package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.subsystems.intake.IntakeConstants.Roller;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public final class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rollerMotor;
    private final TalonFX extensionMotor;

    private Distance setpoint = Meters.of(0.0);

    private final PositionVoltage positionControl;

    private final StatusSignal<Angle>           rollerPosition;
    private final StatusSignal<Temperature>     rollerTemperature;
    private final StatusSignal<Voltage>         rollerVoltage;
    private final StatusSignal<Current>         rollerCurrent;
    private final StatusSignal<AngularVelocity> rollerVelocity;

    private final StatusSignal<Angle>           extensionPosition;
    private final StatusSignal<Temperature>     extensionTemperature;
    private final StatusSignal<Voltage>         extensionVoltage;
    private final StatusSignal<Current>         extensionCurrent;
    private final StatusSignal<AngularVelocity> extensionVelocity;
    private final StatusSignal<Current>         extensionTorqueCurrent;

    public IntakeIOTalonFX(int rollerMotorId, int extensionMotorId) {
        rollerMotor = new TalonFX(rollerMotorId);
        extensionMotor = new TalonFX(extensionMotorId);
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);
        positionControl = new PositionVoltage(0.0);

        TalonFXConfiguration extensionConfig = new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(Extension.GEARING));

        extensionConfig.Slot0 = new Slot0Configs()
                .withKP(Extension.TALONFX_PID.kP)
                .withKI(Extension.TALONFX_PID.kI)
                .withKD(Extension.TALONFX_PID.kD);

        extensionConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        extensionConfig.withCurrentLimits(new CurrentLimitsConfigs()
                                                  .withSupplyCurrentLimit(Extension.CURRENT_LIMIT)
                                                  .withSupplyCurrentLimitEnable(true));
        extensionMotor.getConfigurator().apply(extensionConfig);

        rollerMotor.getConfigurator().apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

        rollerMotor.getConfigurator().apply(new CurrentLimitsConfigs()
                                                    .withSupplyCurrentLimit(Roller.CURRENT_LIMIT)
                                                    .withSupplyCurrentLimitEnable(true));

        extensionPosition = extensionMotor.getPosition();
        extensionTemperature = extensionMotor.getDeviceTemp();
        extensionVoltage = extensionMotor.getMotorVoltage();
        extensionCurrent = extensionMotor.getSupplyCurrent();
        extensionVelocity = extensionMotor.getVelocity();
        extensionTorqueCurrent = extensionMotor.getTorqueCurrent();

        rollerPosition = rollerMotor.getPosition();
        rollerTemperature = rollerMotor.getDeviceTemp();
        rollerVoltage = rollerMotor.getMotorVoltage();
        rollerCurrent = rollerMotor.getSupplyCurrent();
        rollerVelocity = rollerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(

                Extension.UPDATE_FREQUENCY,
                extensionPosition,
                extensionTemperature,
                extensionVoltage,
                extensionCurrent,

                rollerPosition,
                rollerTemperature,
                rollerVoltage,
                rollerCurrent
        );

        extensionMotor.setPosition(0);

        rollerMotor.optimizeBusUtilization();
        extensionMotor.optimizeBusUtilization();
    }

    /**
     * Sets the voltage of the roller motor
     *
     * @param voltage The voltage to set the roller motor to, in volts.
     */
    public void setRollerVoltage(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    /**
     * Sets the voltage of the extension motor
     *
     * @param voltage The voltage to set the extension motor to, in volts.
     */
    public void setExtensionVoltage(double voltage) {
        extensionMotor.setVoltage(voltage);
    }

    /**
     * Moves intake to given setpoint
     *
     * @param position The position to set the extension motor to given position, in meters.
     */
    public void setSetpoint(Distance position) {
        setpoint = position;
        Logger.recordOutput("Intake/Setpoint", position);
        position = Meters.of(MathUtil.clamp(
                position.in(Meters) / Extension.UNIT_CONVERSION_FACTOR, 0,
                Extension.EXTENSION_MAX_DISTANCE.in(Meters) / Extension.UNIT_CONVERSION_FACTOR));
        extensionMotor.setControl(positionControl.withPosition(position.in(Meters)).withSlot(0));
    }

    /**
     * Sets motor to coastMode
     *
     */
    public void coastMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        extensionMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Sets motor to brakeMode
     *
     */
    public void brakeMode() {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Stops the motor
     */
    public void stopMotor() {
        rollerMotor.set(0.0);
        extensionMotor.set(0.0);
    }

    public Distance getSetpoint() {
        return setpoint;
    }

    /**
     * Gets current motor position
     *
     * @return The current position of the intake extension, in meters.
     */
    public Distance getPosition() {
        return Meters.of(extensionPosition.getValueAsDouble() * Extension.UNIT_CONVERSION_FACTOR);
    }

    

    /**
     * Updates the inputs of the intake subsystem
     *
     * @param inputs The inputs object to update with the latest sensor values and other relevant information.
     */
    @Override
    public void updateInputs(IntakeIO.IntakeInputs inputs) {
        inputs.isExtensionConnected = BaseStatusSignal.refreshAll(
                extensionPosition,
                extensionTemperature,
                extensionVoltage,
                extensionCurrent,
                extensionVelocity
        ).isOK();

        inputs.extensionVolts = extensionVoltage.getValue();

        inputs.extensionCurrent = Amps.of(extensionCurrent.getValueAsDouble());
        inputs.extensionTorqueCurrent = Amps.of(extensionTorqueCurrent.getValueAsDouble());
        inputs.extensionTemp = extensionTemperature.getValueAsDouble();

        inputs.extensionPosition = getPosition();

        inputs.extensionVelocity = MetersPerSecond.of(extensionVelocity.getValueAsDouble());
        inputs.isExtensionRunning = Math.abs(extensionVoltage.getValueAsDouble()) > 0.1;

        inputs.isRollerConnected = BaseStatusSignal.refreshAll(
                rollerPosition,
                rollerTemperature,
                rollerVoltage,
                rollerCurrent,
                rollerVelocity
        ).isOK();

        inputs.rollerVolts = Volts.of(rollerVoltage.getValueAsDouble());
        inputs.rollerCurrent = Amps.of(rollerCurrent.getValueAsDouble());
        inputs.rollerTemp = rollerTemperature.getValueAsDouble();
        inputs.rollerVelocity = RotationsPerSecond.of(rollerVelocity.getValueAsDouble());
        inputs.extensionSetpoint = setpoint;
    }
}