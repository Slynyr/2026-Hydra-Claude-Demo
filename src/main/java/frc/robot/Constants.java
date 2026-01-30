// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean IS_TUNING = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class kAutoAlign{
    public static final PIDConstants ALIGN_PID = new PIDConstants(4.9, 0.0, 0.28);

    public static final Distance TRANSLATION_TOLERANCE;
    public static final Angle    ROTATION_TOLERANCE   ;

    public static final LinearVelocity VELOCITY_TOLERANCE = MetersPerSecond.of(0.18);
    public static final LinearVelocity AUTO_VELOCITY_TOLERANCE = MetersPerSecond.of(0.15);
    public static final AngularVelocity AUTO_ANGULAR_VELOCITY_TOLERANCE = DegreesPerSecond.of(0.15);
    public static final AngularVelocity ANGULAR_VELOCITY_TOLERANCE = DegreesPerSecond.of(0.18);

    static {
        if (IS_TUNING) {
            TRANSLATION_TOLERANCE = Centimeters.of(0.00);
            ROTATION_TOLERANCE    = Degrees    .of(0.00);
        } else {
            TRANSLATION_TOLERANCE = Centimeters.of(2.00);
            ROTATION_TOLERANCE    = Degrees    .of(1.25);
        }
    }

  }

  public static final class kBump{
    public static final double BUMP_SPEED_MODIFIER = 0.33;
  }
}
