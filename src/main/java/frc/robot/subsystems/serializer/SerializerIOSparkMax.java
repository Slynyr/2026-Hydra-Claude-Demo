package frc.robot.subsystems.serializer;

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
 * Indexer for Ortona
 */
public class SerializerIOSparkMax implements SerializerIO {
    private final SparkMax indexerMotor;
    private final SparkMax feederMotor;

    public SerializerIOSparkMax(int indexerMotorID, int feederMotorID) {
        indexerMotor = new SparkMax(indexerMotorID, MotorType.kBrushless);
        feederMotor = new SparkMax(feederMotorID, MotorType.kBrushless);
        SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();
        SparkMaxConfig feederMotorConfig = new SparkMaxConfig();

        indexerMotorConfig.smartCurrentLimit((int)SerializerConstants.ORTONA_SPARK_MAX_CURRENT_LIMIT.magnitude());
        indexerMotorConfig.idleMode(IdleMode.kBrake);
        indexerMotorConfig.inverted(SerializerConstants.ORTONA_INDEXER_MOTOR_INVERTED);

        feederMotorConfig.smartCurrentLimit((int)SerializerConstants.ORTONA_SPARK_MAX_CURRENT_LIMIT.magnitude());
        feederMotorConfig.idleMode(IdleMode.kBrake);
        feederMotorConfig.inverted(SerializerConstants.ORTONA_FEEDER_MOTOR_INVERTED);

        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIndexerMotorVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    @Override
    public void setFeederMotorVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    @Override
    public void updateInputs(SerializerInputs inputs) {
        inputs.isIndexerMotorConnected = !(indexerMotor.getFaults().motorType || indexerMotor.getFaults().can);
        inputs.indexerAppliedVoltage = Volts.of(indexerMotor.get() * RobotController.getBatteryVoltage());
        inputs.indexerAppliedCurrent = Amps.of(indexerMotor.getOutputCurrent());
        inputs.indexerMotorTemperature = indexerMotor.getMotorTemperature();

        inputs.isFeederMotorConnected = !(feederMotor.getFaults().motorType || indexerMotor.getFaults().can);
        inputs.feederAppliedVoltage = Volts.of(feederMotor.get() * RobotController.getBatteryVoltage());
        inputs.feederAppliedCurrent = Amps.of(feederMotor.getOutputCurrent());
        inputs.feederMotorTemperature = feederMotor.getMotorTemperature();
    }
}
