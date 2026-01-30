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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean isDriveConnected = false;
    public Angle drivePositionRad = Radians.of(0.0);
    public AngularVelocity driveVelocityRadPerSec = RadiansPerSecond.of(0.0);
    public Voltage driveAppliedVolts = Volts.of(0.0);
    public Current driveCurrentAmps = Amps.of(0.0);

    public boolean isTurnConnected = false;
    public boolean turnEncoderConnected = false;
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public AngularVelocity turnVelocityRadPerSec = RadiansPerSecond.of(0.0);
    public Voltage turnAppliedVolts = Volts.of(0.0);
    public Current turnCurrentAmps = Amps.of(0.0);

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    public MagnetHealthValue magnetHealth = MagnetHealthValue.Magnet_Invalid;

  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  public default void driveNeutralMode(NeutralModeValue mode) {}

  public default void steerNeutralMode(NeutralModeValue mode) {}

}
