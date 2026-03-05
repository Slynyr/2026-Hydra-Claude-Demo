package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Volts;

public class LauncherIOTalonFX implements LauncherIO {
    // Motors and sensors
    private final TalonFX     leaderMotor;
    private final Servo       hoodServo;
    private final Servo       hoodServo2;
    private final AnalogInput ultrasonic;

    private double servo1CurPos;
    private double servo2CurPos;
    private double servo1Setpoint;
    private double servo2Setpoint;

    // status signals
    private final StatusSignal<MagnetHealthValue> magnetHealth;

    private final StatusSignal<Temperature>     launcherTemp;
    private final StatusSignal<Voltage>         launcherVoltage;
    private final StatusSignal<Current>         launcherCurrent;
    private final StatusSignal<AngularVelocity> launcherVelocity;

    private final StatusSignal<Temperature>     launcherFollowerTemp;
    private final StatusSignal<Voltage>         launcherFollowerVoltage;
    private final StatusSignal<Current>         launcherFollowerCurrent;
    private final StatusSignal<AngularVelocity> launcherFollowerVelocity;

    public LauncherIOTalonFX(
            int launcherCANCoderID,
            int launcherCanID,
            int launcherFollowerCanID,
            int ultrasonicChannel,
            int servoChannel,
            int servoChannel2
    ) {
        // Motors and sensors
        leaderMotor = new TalonFX(launcherCanID);
        TalonFX followerMotor = new TalonFX(launcherFollowerCanID);
        CANcoder encoder = new CANcoder(launcherCANCoderID);
        hoodServo = new Servo(servoChannel);
        hoodServo2 = new Servo(servoChannel2);
        ultrasonic = new AnalogInput(ultrasonicChannel);

        // IOs
        magnetHealth = encoder.getMagnetHealth();

        launcherTemp = leaderMotor.getDeviceTemp();
        launcherVoltage = leaderMotor.getMotorVoltage();
        launcherCurrent = leaderMotor.getSupplyCurrent();
        launcherVelocity = leaderMotor.getVelocity();

        launcherFollowerTemp = followerMotor.getDeviceTemp();
        launcherFollowerVoltage = followerMotor.getMotorVoltage();
        launcherFollowerCurrent = followerMotor.getSupplyCurrent();
        launcherFollowerVelocity = followerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                magnetHealth,

                launcherTemp,
                launcherVoltage,
                launcherCurrent,
                launcherVelocity,

                launcherFollowerTemp,
                launcherFollowerVoltage,
                launcherFollowerCurrent,
                launcherFollowerVelocity
        );

        // Configurators
        hoodServo.setBoundsMicroseconds(
                LauncherConstants.Hood.MAX_PULSE_WIDTH,
                LauncherConstants.Hood.SERVO_DEADBAND_MAX,
                LauncherConstants.Hood.SERVO_DEADBAND_CENTER,
                LauncherConstants.Hood.SERVO_DEADBAND_MIN,
                LauncherConstants.Hood.MIN_PULSE_WIDTH);
        hoodServo2.setBoundsMicroseconds(
                LauncherConstants.Hood.MAX_PULSE_WIDTH,
                LauncherConstants.Hood.SERVO_DEADBAND_MAX,
                LauncherConstants.Hood.SERVO_DEADBAND_CENTER,
                LauncherConstants.Hood.SERVO_DEADBAND_MIN,
                LauncherConstants.Hood.MIN_PULSE_WIDTH);

        TalonFXConfigurator leaderConfig = leaderMotor.getConfigurator();
        TalonFXConfigurator followerConfig = followerMotor.getConfigurator();

        encoder.getConfigurator()
                        .apply(new CANcoderConfiguration().MagnetSensor
                                       .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

        // Slot configs
        Slot0Configs motorTuning = new Slot0Configs()
                .withKP(LauncherConstants.Launcher.PID.getP())
                .withKI(LauncherConstants.Launcher.PID.getI())
                .withKD(LauncherConstants.Launcher.PID.getD())
                .withKV(LauncherConstants.Launcher.kV)
                .withKS(LauncherConstants.Launcher.kS)
                .withKG(LauncherConstants.Launcher.kG);
        leaderConfig.apply(motorTuning);
        followerConfig.apply(motorTuning);

        // Current limit configs
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(LauncherConstants.Launcher.SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        leaderConfig.apply(currentLimits);
        followerConfig.apply(currentLimits);

        // Motor output configs
        leaderConfig.apply(new MotorOutputConfigs()
                                   .withNeutralMode(NeutralModeValue.Coast)
                                   .withInverted(InvertedValue.CounterClockwise_Positive));
        followerConfig.apply(new MotorOutputConfigs()
                                     .withNeutralMode(NeutralModeValue.Coast));

        // Feedback configs
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withRotorToSensorRatio(LauncherConstants.Launcher.MOTOR_ENCODER_GEAR_RATIO)
                .withRemoteCANcoder(encoder);
        leaderConfig.apply(feedbackConfigs);
        followerConfig.apply(feedbackConfigs);

        followerMotor.setControl(new Follower(launcherCanID, MotorAlignmentValue.Opposed));
    }

    // Run systems
    @Override
    public void runVelocity(Supplier<AngularVelocity> velocity) {
        leaderMotor.setControl(
                new VelocityVoltage(velocity.get())
                        .withSlot(0)
                        .withFeedForward(0));
    }

    @Override
    public void updateHood(Distance extension) {
        double targetSetpoint = extension.in(Millimeters);
        double t = Timer.getFPGATimestamp();

        // update servo continuous positions m
        if (servo1CurPos > servo1Setpoint + 30 * t) servo1CurPos -= 30 * t;
        else if (servo1CurPos < servo1Setpoint - 30 * t) servo1CurPos += 30 * t;
        else servo1CurPos = servo1Setpoint;

        if (servo2CurPos > servo2Setpoint + 30 * t) servo2CurPos -= 30 * t;
        else if (servo2CurPos < servo2Setpoint - 30 * t) servo2CurPos += 30 * t;
        else servo2CurPos = servo2Setpoint;

        // update applied setpoints for servos
        double appliedSetpoint = MathUtil.clamp(
                targetSetpoint, 0, LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters));
        servo1Setpoint = appliedSetpoint;
        appliedSetpoint = (targetSetpoint / LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters) * 2) - 1;
        hoodServo.setSpeed(appliedSetpoint);

        appliedSetpoint = MathUtil.clamp(targetSetpoint, 0, LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters));
        servo2Setpoint = appliedSetpoint;
        appliedSetpoint = (targetSetpoint / LauncherConstants.Hood.MAX_EXTENSION.in(Millimeters) * 2) - 1;
        hoodServo2.setSpeed(appliedSetpoint);
    }

    @Override
    public Distance getHoodExtension() {
        return Millimeters.of(hoodServo.getPosition());
    }

    @Override
    public Voltage getUltrasonicVolts() {
        return Volts.of(ultrasonic.getVoltage());
    }

    /**
     * @return the velocity of the LEADER motor, this ignores the follower.
     */
    @Override
    public AngularVelocity getVelocity() {
        return launcherVelocity.getValue();
    }

    // Stops
    @Override
    public void stopLauncher() {
        leaderMotor.setVoltage(0);
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        // Launcher
        inputs.isCANCoderConnected = BaseStatusSignal.refreshAll(magnetHealth).isOK();
        inputs.magnetHealth = magnetHealth.getValue();

        inputs.isLauncherConnected = BaseStatusSignal.refreshAll(
                launcherVoltage,
                launcherCurrent,
                launcherTemp,
                launcherVelocity
        ).isOK();
        inputs.launcherTemperature = launcherTemp.getValueAsDouble();
        inputs.launcherVoltage = launcherVoltage.getValue();
        inputs.launcherCurrent = launcherCurrent.getValue();
        inputs.launcherVelocity = launcherVelocity.getValue();

        inputs.isLauncherFollowerConnected = BaseStatusSignal.refreshAll(
                launcherFollowerVoltage,
                launcherFollowerCurrent,
                launcherFollowerTemp,
                launcherFollowerVelocity
        ).isOK();
        inputs.launcherFollowerTemperature = launcherFollowerTemp.getValueAsDouble();
        inputs.launcherFollowerVoltage = launcherFollowerVoltage.getValue();
        inputs.launcherFollowerCurrent = launcherFollowerCurrent.getValue();
        inputs.launcherFollowerVelocity = launcherFollowerVelocity.getValue();

        // Hood
        inputs.hoodServo1Pos = Millimeters.of(servo1CurPos);
        inputs.hoodServo2Pos = Millimeters.of(servo2CurPos);
        inputs.hoodServo1Target = Millimeters.of(servo1Setpoint);
        inputs.hoodServo2Target = Millimeters.of(servo2Setpoint);

        inputs.ultrasonicVoltage = getUltrasonicVolts();
    }
}
