package frc.robot.subsystems.feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.RobotController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

/**
 * Feeder for Ortona
 */
public class FeederIOSparkMax implements FeederIO {
    private final SparkMax feederMotor;

    public FeederIOSparkMax(int feederMotorID) {
        feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
        SparkMaxConfig feederMotorConfig = new SparkMaxConfig();

        feederMotorConfig.smartCurrentLimit((int)FeederConstants.ORTONA_SPARK_MAX_CURRENT_LIMIT.magnitude());
        feederMotorConfig.idleMode(IdleMode.kBrake);
        feederMotorConfig.inverted(FeederConstants.ORTONA_FEEDER_MOTOR_INVERTED);
        feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setMotorVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.isMotorConnected = !(feederMotor.getFaults().motorType || feederMotor.getFaults().can);
        inputs.appliedVoltage = Volts.of(feederMotor.get() * RobotController.getBatteryVoltage());
        inputs.appliedCurrent = Amps.of(feederMotor.getOutputCurrent());
        inputs.motorTemperature = feederMotor.getMotorTemperature();
    }
}
