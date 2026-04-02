// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean isConnected = false;

        public Rotation2d yawPosition   = Rotation2d.kZero;
        public Rotation2d pitchPosition = Rotation2d.kZero;
        public Rotation2d rollPosition  = Rotation2d.kZero;

        public AngularVelocity yawVelocity   = RadiansPerSecond.of(0.0);
        public AngularVelocity pitchVelocity = RadiansPerSecond.of(0.0);
        public AngularVelocity rollVelocity  = RadiansPerSecond.of(0.0);

        public double[] odometryYawTimestamps   = new double[]{};
        public double[] odometryPitchTimestamps = new double[]{};
        public double[] odometryRollTimestamps  = new double[]{};

        public Rotation2d[] odometryYawPositions   = new Rotation2d[]{};
        public Rotation2d[] odometryPitchPositions = new Rotation2d[]{};
        public Rotation2d[] odometryRollPositions  = new Rotation2d[]{};

        public Angle tilt = Degrees.of(0.0);
    }

    default void updateInputs(GyroIOInputs inputs) {}

    /**
     * Zeros the gyroscope or IMU
     */
    default void zero() {}

    /**
     * Sets the yaw of the gyroscope or IMU
     * @param yaw new yaw
     */
    default void setYaw(Rotation2d yaw) {}
}
