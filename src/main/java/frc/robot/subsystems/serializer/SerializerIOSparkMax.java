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

public class SerializerIOSparkMax implements SerializerIO {

    private final SparkMax indexerMotor;

    public SerializerIOSparkMax(int indexerMotorID) {

        indexerMotor = new SparkMax(indexerMotorID, MotorType.kBrushless);
        SparkMaxConfig indexerMotorConfig = new SparkMaxConfig();

        indexerMotorConfig.smartCurrentLimit((int)SerializerConstants.ORTONA_SPARK_MAX_CURRENT_LIMIT.magnitude());
        indexerMotorConfig.idleMode(IdleMode.kBrake);
        indexerMotorConfig.inverted(SerializerConstants.ORTONA_INDEXER_MOTOR_INVERTED);

        indexerMotor.configure(indexerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void setVoltage(double voltage) {

        indexerMotor.setVoltage(voltage);

    }


    @Override
    public void updateInputs(SerializerInputs inputs) {

        inputs.isIndexerMotorConnected = !(indexerMotor.getFaults().motorType || indexerMotor.getFaults().can);
        inputs.indexerAppliedVoltage = Volts.of(indexerMotor.get() * RobotController.getBatteryVoltage());
        inputs.indexerAppliedCurrent = Amps.of(indexerMotor.getOutputCurrent());
        inputs.indexerMotorTemperature = indexerMotor.getMotorTemperature();

    }
}
