// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants.Tower;

import static edu.wpi.first.units.Units.*;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running on a roboRIO. Change
 * the value of "simMode" to switch between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static final boolean IS_TUNING = false;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class DeviceID {
        // VISION
        public static final String LIMELIGHT_NAME = "limelight";

        // INTAKE
        public static final int INTAKE_CAN_RANGE       = 20;
        public static final int INTAKE_EXTENSION_MOTOR = 28;
        public static final int INTAKE_ROLLER_MOTOR    = 29;

        // LAUNCHER
        public static final int LAUNCHER_CAN_RANGE          = 21;
        public static final int LAUNCHER_MOTOR_1            = 24;
        public static final int LAUNCHER_MOTOR_2            = 25;
        public static final int LAUNCHER_FLYWHEEL_CAN_CODER = 26;
        public static final int LAUNCHER_HOOD_SERVO_1       = 0; // TODO
        public static final int LAUNCHER_HOOD_SERVO_2       = 0; // TODO

        // FEEDER
        public static final int FEEDER_MOTOR_TOP    = 0; // TODO: jaden will update these
        public static final int FEEDER_MOTOR_BOTTOM = 0;

        // HOPPER
        public static final int HOPPER_MAIN_MOTOR_ID     = 20;
        public static final int HOPPER_FOLLOWER_MOTOR_ID = 21;

        // SERIALIZER
        public static final int SERIALIZER_MOTOR = 31;

        // CLIMBER
        public static final int CLIMBER_MOTOR = 0; // TODO: yonina will update this
    }

    public static final class kAutoAlign {
        public static final PIDConstants ALIGN_PID = new PIDConstants(4.9, 0.0, 0.28);

        // Blue HUB Position relative to a blue alliance origin
        public static final Pose2d HUB_POSE = new Pose2d(
                new Translation2d(Meters.of(4.620), Meters.of(4.030)), Rotation2d.kZero);

        public static final Distance TRANSLATION_TOLERANCE;
        public static final Angle    ROTATION_TOLERANCE;

        public static final Distance TRANSLATION_TOLERANCE_CLIMB_PREP;
        public static final Angle    ROTATION_TOLERANCE_CLIMB_PREP;

        public static final LinearVelocity VELOCITY_TOLERANCE            = MetersPerSecond.of(0.18);
        public static final LinearVelocity AUTO_VELOCITY_TOLERANCE       = MetersPerSecond.of(0.15);
        //Tune to allow the climber prep pose to only affect approach angle + keep velocity
        public static final LinearVelocity VELOCITY_TOLERANCE_CLIMB_PREP = MetersPerSecond.of(1);

        public static final AngularVelocity AUTO_ANGULAR_VELOCITY_TOLERANCE = DegreesPerSecond.of(0.15);
        public static final AngularVelocity ANGULAR_VELOCITY_TOLERANCE      = DegreesPerSecond.of(0.18);

        static {
            if (IS_TUNING) {
                TRANSLATION_TOLERANCE = Centimeters.of(0.00);
                ROTATION_TOLERANCE = Degrees.of(0.00);

                TRANSLATION_TOLERANCE_CLIMB_PREP = Centimeter.of(0.0);
                ROTATION_TOLERANCE_CLIMB_PREP = Degrees.of(0.00);
            } else {
                TRANSLATION_TOLERANCE = Centimeters.of(2.00);
                ROTATION_TOLERANCE = Degrees.of(1.25);

                //Tune to allow the climber prep pose to only affect approach angle
                TRANSLATION_TOLERANCE_CLIMB_PREP = Centimeters.of(2.00);
                ROTATION_TOLERANCE_CLIMB_PREP = Degrees.of(1.25);
            }
        }

        public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY     = MetersPerSecond.of(2.75);
        public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION = MetersPerSecondPerSecond.of(16);

        public static final LinearVelocity     MAX_AUTO_ALIGN_VELOCITY_CLIMB     = MetersPerSecond.of(1);
        public static final LinearAcceleration MAX_AUTO_ALIGN_ACCELERATION_CLIMB = MetersPerSecondPerSecond.of(8);

        // Distance from the upright to the robot climber
        public static final Distance CLIMBER_DISTANCE_FROM_UPRIGHT = Meters.of(
                (Tower.width - Tower.innerOpeningWidth) / 2);
    }

    /*
     * Passing positon if looking from alliance driver station
     */
    public static enum PassingPositions {
        RIGHT(new Pose2d(new Translation2d(Meters.of(2.5), Meters.of(1.26)), Rotation2d.kZero)),
        MIDDLE(new Pose2d(new Translation2d(Meters.of(2.1), Meters.of(3.95)), Rotation2d.kZero)),
        LEFT(new Pose2d(new Translation2d(Meters.of(2.5), Meters.of(6.8)), Rotation2d.kZero));

        Pose2d pose;

        private PassingPositions(Pose2d pose) {
            this.pose = pose;
        }
    }

    /*
     * Climibing position if looking from alliance driver station
     */
    public static enum ClimbingPositions {
        // RIGHT   (new Pose2d(new Translation2d(Meters.of(1.15), Meters.of(2.66)), Rotation2d.kZero)),
        // LEFT    (new Pose2d(new Translation2d(Meters.of(1.15), Meters.of(4.84)), Rotation2d.k180deg)),
        RIGHT(new Pose2d(
                new Translation2d(
                        Meters.of(Tower.rightUpright.getX()),
                        Meters.of(
                                Tower.rightUpright.getY() - (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) -
                                kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters)
                        )
                ),
                Rotation2d.kZero)),
        LEFT(new Pose2d(
                new Translation2d(
                        Meters.of(Tower.leftUpright.getX()),
                        Meters.of(
                                Tower.leftUpright.getY() + (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) +
                                kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters)
                        )
                ),
                Rotation2d.k180deg)
        ),

        LEFT_PREP(new Pose2d(
                new Translation2d(Meters.of(Tower.leftUpright.getX()), Meters.of(5.00)),
                Rotation2d.k180deg)),
        RIGHT_PREP(new Pose2d(
                new Translation2d(Meters.of(Tower.rightUpright.getX()), Meters.of(2.450)),
                Rotation2d.kZero));

        Pose2d pose;

        private ClimbingPositions(Pose2d pose) {
            this.pose = pose;
        }
    }

    public static final class kBump {
        // Percentage of max speed
        public static final double BUMP_SPEED_MODIFIER = 0.4;
    }
}
