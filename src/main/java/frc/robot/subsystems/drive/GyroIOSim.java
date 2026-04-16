package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyro;

    public GyroIOSim(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.isConnected = true;

        inputs.yawPosition = gyro.getGyroReading();
        inputs.yawVelocity = RadiansPerSecond.of(gyro.getMeasuredAngularVelocity().in(RadiansPerSecond));
        inputs.odometryYawTimestamps = new double[]{};
        inputs.odometryYawPositions = gyro.getCachedGyroReadings();
    }

    @Override
    public void zero(){
        setYaw(Rotation2d.k180deg);
    }

    @Override
    public void setYaw(Rotation2d rotation2d){
        gyro.setRotation(rotation2d);
    }
}
