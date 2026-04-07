// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean         isDriveConnected  = false;
        public Angle           drivePositionRad  = Radians.of(0.0);
        public AngularVelocity driveVelocity     = RadiansPerSecond.of(0.0);
        public Voltage         driveAppliedVolts = Volts.of(0.0);
        public Current         driveCurrentAmps  = Amps.of(0.0);
        public Current         driveSupplyCurrentAmps  = Amps.of(0.0);

        public boolean         isTurnMotorConnected   = false;
        public boolean         isTurnEncoderConnected = false;
        public Rotation2d      turnAbsolutePosition   = Rotation2d.kZero;
        public Rotation2d      turnPosition           = Rotation2d.kZero;
        public AngularVelocity turnVelocity           = RadiansPerSecond.of(0.0);
        public Voltage         turnAppliedVolts       = Volts.of(0.0);
        public Current         turnCurrentAmps        = Amps.of(0.0);
        public Current         turnSupplyCurrentAmps        = Amps.of(0.0);

        public double[]     odometryTimestamps        = new double[]{};
        public double[]     odometryDrivePositionsRad = new double[]{};
        public Rotation2d[] odometryTurnPositions     = new Rotation2d[]{};

        public MagnetHealthValue magnetHealth = MagnetHealthValue.Magnet_Invalid;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    default void setDriveVelocity(double velocityRadPerSec) {}

    default void setTurnVoltage(double voltage) {}

    /** Run the turn motor to the specified rotation. */
    default void setTurnPosition(Rotation2d rotation) {}

    default void setTurnVelocity(AngularVelocity velocity) {}

    default void driveNeutralMode(NeutralModeValue mode) {}

    default void steerNeutralMode(NeutralModeValue mode) {}
}
