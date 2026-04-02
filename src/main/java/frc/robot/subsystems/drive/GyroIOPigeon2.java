// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;
import java.util.Queue;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

    private final StatusSignal<Angle> yaw   = pigeon.getYaw();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll  = pigeon.getRoll();

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;

    private final Queue<Double> pitchPositionQueue;
    private final Queue<Double> pitchTimestampQueue;

    private final Queue<Double> rollPositionQueue;
    private final Queue<Double> rollTimestampQueue;

    private final StatusSignal<AngularVelocity> yawVelocity   = pigeon.getAngularVelocityZWorld();
    // TODO: DOUBLE CHECK TO SEE IF CORRECT: Pigeon 2.0 User Manual Page 20
    private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityYDevice();
    private final StatusSignal<AngularVelocity> rollVelocity  = pigeon.getAngularVelocityXDevice();

    private final StatusSignal<Boolean> motionStackLate = pigeon.getFault_DataAcquiredLate();

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(Objects.requireNonNullElseGet(
                TunerConstants.DrivetrainConstants.Pigeon2Configs,
                Pigeon2Configuration::new));

        pigeon.getConfigurator().setYaw(0.0);

        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
        pitch.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
        roll.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);

        yawVelocity.setUpdateFrequency(DriveConstants.ODOMETRY_VELOCITY_UPDATE_FREQUENCY);
        pitchVelocity.setUpdateFrequency(DriveConstants.ODOMETRY_VELOCITY_UPDATE_FREQUENCY);
        rollVelocity.setUpdateFrequency(DriveConstants.ODOMETRY_VELOCITY_UPDATE_FREQUENCY);
        motionStackLate.setUpdateFrequency(DriveConstants.ODOMETRY_VELOCITY_UPDATE_FREQUENCY);

        pigeon.optimizeBusUtilization();

        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());

        pitchTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pitch.clone());

        rollTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        rollPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(roll.clone());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        Logger.recordOutput("Gyro/Faults/MotionDataStackLate", motionStackLate.getValue());

        inputs.isConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity)
                                             .equals(StatusCode.OK);

        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());

        inputs.yawVelocity = RadiansPerSecond.of(Units.degreesToRadians(yawVelocity.getValueAsDouble()));
        inputs.pitchVelocity = RadiansPerSecond.of(Units.degreesToRadians(pitchVelocity.getValueAsDouble()));
        inputs.rollVelocity = RadiansPerSecond.of(Units.degreesToRadians(rollVelocity.getValueAsDouble()));

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryPitchTimestamps =
                pitchTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryRollTimestamps =
                rollTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();

        inputs.odometryYawPositions =
                yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        inputs.odometryPitchPositions =
                pitchPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        inputs.odometryRollPositions =
                rollPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();

        pitchTimestampQueue.clear();
        pitchPositionQueue.clear();

        rollTimestampQueue.clear();
        rollPositionQueue.clear();

        inputs.tilt = Radians.of(
                Math.acos(Math.cos(inputs.pitchPosition.getRadians()) * Math.cos(inputs.rollPosition.getRadians())));
    }

    @Override
    public void zero() {
        pigeon.setYaw(180);
    }

    @Override
    public void setYaw(Rotation2d yaw) {
        pigeon.setYaw(yaw.getDegrees());
    }
}
