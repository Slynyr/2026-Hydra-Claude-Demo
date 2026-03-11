// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants;

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
        public static final int LAUNCHER_CANCODER     = 26;
        public static final int LAUNCHER_MOTOR_1      = 24;
        public static final int LAUNCHER_MOTOR_2      = 25;
        public static final int LAUNCHER_HOOD_SERVO_1 = 8;
        public static final int LAUNCHER_HOOD_SERVO_2 = 9;
        public static final int LAUNCHER_ULTRASONIC_CHANNEL = 1;


        // FEEDER
        public static final int FEEDER_MOTOR_TOP    = 33; // TODO: jaden will update these
        public static final int FEEDER_MOTOR_BOTTOM = 27;

        // HOPPER
        public static final int HOPPER_MOTOR_ID     = 30;

        // SERIALIZER
        public static final int SERIALIZER_MOTOR = 31;

        // CLIMBER
        public static final int CLIMBER_MOTOR = 32;
    }

    public static final class kAutoAlign {
        public static final PIDConstants ALIGN_PID = 
        new PIDConstants
        (1.75, 
        0.0, 
        0.28);
        // 4.9, 0, 0.28

        public static final Distance TRANSLATION_TOLERANCE;
        public static final Angle    ROTATION_TOLERANCE;
        public static final AngularVelocity    ROTATION_VELOCITY_TOLERANCE;

        public static final Distance TRANSLATION_TOLERANCE_CLIMB_PREP;
        public static final Angle    ROTATION_TOLERANCE_CLIMB_PREP;

        public static final LinearVelocity VELOCITY_TOLERANCE            = MetersPerSecond.of(0.18);
        public static final LinearVelocity AUTO_VELOCITY_TOLERANCE       = MetersPerSecond.of(0.15);
        //Tune to allow the climber prep pose to only affect approach hoodExtension + keep velocity
        public static final LinearVelocity VELOCITY_TOLERANCE_CLIMB_PREP = MetersPerSecond.of(1);

        public static final AngularVelocity AUTO_ANGULAR_VELOCITY_TOLERANCE = DegreesPerSecond.of(0.15);
        public static final AngularVelocity ANGULAR_VELOCITY_TOLERANCE      = DegreesPerSecond.of(0.18);

        static {
            if (IS_TUNING) {
                TRANSLATION_TOLERANCE = Centimeters.of(0.00);
                ROTATION_TOLERANCE = Degrees.of(0.00);
                ROTATION_VELOCITY_TOLERANCE = DegreesPerSecond.of(0.0);

                TRANSLATION_TOLERANCE_CLIMB_PREP = Centimeter.of(0.0);
                ROTATION_TOLERANCE_CLIMB_PREP = Degrees.of(0.00);


            } else {

                TRANSLATION_TOLERANCE = Centimeters.of(4.00);
                ROTATION_TOLERANCE = Degrees.of(1.25);
                ROTATION_VELOCITY_TOLERANCE = DegreesPerSecond.of(10.0);

                //Tune to allow the climber prep pose to only affect approach hoodExtension
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
                (FieldConstants.Tower.width - FieldConstants.Tower.innerOpeningWidth) / 2);
    }

    /*
     * Passing positon if looking from alliance driver station
     */
    public enum PassingPositions {
        RIGHT(new Pose2d(new Translation2d(Meters.of(2.5), Meters.of(1.26)), Rotation2d.kZero)),
        MIDDLE(new Pose2d(new Translation2d(Meters.of(2.1), Meters.of(3.95)), Rotation2d.kZero)),
        LEFT(new Pose2d(new Translation2d(Meters.of(2.5), Meters.of(6.8)), Rotation2d.kZero));

        Pose2d pose;

        PassingPositions(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    /*
     * Climbing position if looking from blue alliance driver station
     */
    public enum ClimbingPositions {
        // RIGHT   (new Pose2d(new Translation2d(Meters.of(1.15), Meters.of(2.66)), Rotation2d.kZero)),
        // LEFT    (new Pose2d(new Translation2d(Meters.of(1.15), Meters.of(4.84)), Rotation2d.k180deg)),
        RIGHT(new Pose2d(
                new Translation2d(
                        Meters.of(FieldConstants.Tower.rightUpright.getX()),
                        Meters.of(
                                FieldConstants.Tower.rightUpright.getY() - (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) -
                                kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters)
                        )
                ),
                Rotation2d.kZero)),
        LEFT(new Pose2d(
                new Translation2d(
                        Meters.of(FieldConstants.Tower.leftUpright.getX()),
                        Meters.of(
                                FieldConstants.Tower.leftUpright.getY() + (DriveConstants.ROBOT_WIDTH.in(Meters) / 2) +
                                kAutoAlign.CLIMBER_DISTANCE_FROM_UPRIGHT.in(Meters)
                        )
                ),
                Rotation2d.k180deg)
        ),

        LEFT_PREP(new Pose2d(
                new Translation2d(Meters.of(FieldConstants.Tower.leftUpright.getX()), Meters.of(5.00)),
                Rotation2d.k180deg)),
        RIGHT_PREP(new Pose2d(
                new Translation2d(Meters.of(FieldConstants.Tower.rightUpright.getX()), Meters.of(2.450)),
                Rotation2d.kZero));

        Pose2d pose;

        ClimbingPositions(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose(){
            return this.pose;
        }
    }

    public static final class kBump {
        // Percentage of max speed
        public static final double         BUMP_SPEED_MODIFIER  = 0.4;
        // TODO: GET A REAL NUMBER NOT A GUESS
        public static final LinearVelocity BUMP_TRAVERSAL_SPEED = FeetPerSecond.of(5.5);
        public static final Time           SETTLING_TIME        = Seconds.of(1);
    }

    public static final class GameCommandsConstants{
        public static final Time        WAIT_TIME_BEFORE_AGITATE            = Milliseconds.of(500);
        public static final Time        WAIT_TIME_BEFORE_INTAKE_EXTENSION   = Milliseconds.of(500);
        public static final Time        AUTO_LAUNCH_WAIT_TIME               = Seconds.of(5);
        public static final Distance    EXTEND_POINT                         = Centimeters.of(23.5);
        public static final Distance    RETRACT_POINT                        = EXTEND_POINT.minus(Centimeters.of(7.5));
        // TODO: TUNE THIS VALUE
        public static final AngularVelocity PASSING_RPS                     = RotationsPerSecond.of(50);
        public static final Distance PASSING_HOOD_ANGLE                     = Millimeters.of(50);
    }

    public static final class kField{
        public static final Pose2d BLUE_HUB 				=	new Pose2d(
                                                                    new Translation2d(
                                                                            FieldConstants.Hub.topCenterPoint.getMeasureX(),
                                                                            FieldConstants.Hub.topCenterPoint.getMeasureY()),
                                                                    Rotation2d.kZero
                                                                );

        public static final Pose2d RED_HUB					=	new Pose2d(
                                                                    new Translation2d(
                                                                            FieldConstants.Hub.oppTopCenterPoint.getMeasureX(),
                                                                            FieldConstants.Hub.oppTopCenterPoint.getMeasureY()),
                                                                    Rotation2d.kZero
                                                                );

        public static final Rectangle2d NEUTRAL_ZONE 		=	new Rectangle2d(
                                                                    new Translation2d(
                                                                        FieldConstants.LeftTrench.openingTopLeft.getMeasureX(), 
                                                                        FieldConstants.LeftTrench.openingTopLeft.getMeasureY() 
                                                                    ), 
                                                                    new Translation2d(
                                                                        FieldConstants.RightTrench.oppOpeningTopRight.getMeasureX(),
                                                                        FieldConstants.RightTrench.oppOpeningTopRight.getMeasureY()
                                                                ));

        public static final Rectangle2d BLUE_ALLIANCE_ZONE 	=	new Rectangle2d(
                                                                    new Translation2d(
                                                                        Meters.of(0.0), 
                                                                        Meters.of(FieldConstants.fieldWidth)
                                                                    ), 
                                                                    new Translation2d(
                                                                        FieldConstants.RightTrench.openingTopRight.getMeasureX(),
                                                                        FieldConstants.RightTrench.openingTopRight.getMeasureY()
                                                                ));

        public static final Rectangle2d RED_ALLIANCE_ZONE 	= 	new Rectangle2d(
                                                                    new Translation2d(
                                                                        Meters.of(FieldConstants.fieldLength), 
                                                                        Meters.of(FieldConstants.fieldWidth)
                                                                    ), 
                                                                    new Translation2d(
                                                                        FieldConstants.RightTrench.oppOpeningTopRight.getMeasureX(),
                                                                        FieldConstants.RightTrench.oppOpeningTopRight.getMeasureY()
                                                                ));

        /**
         * Left half of field if looking from blue alliance drive station
         */
        public static final Rectangle2d LEFT_HALF         	=  new Rectangle2d(
                                                                    new Translation2d(
                                                                        Meters.of(0.0), 
                                                                        Meters.of(FieldConstants.fieldWidth)
                                                                    ),
                                                                    new Translation2d(
                                                                        Meters.of(FieldConstants.fieldLength),
                                                                        Meters.of(FieldConstants.LinesHorizontal.center)
                                                                ));

        /**
         * Right half of field if looking from blue alliance drive station
         */
        public static final Rectangle2d RIGHT_HALF         	=  new Rectangle2d(
                                                                    new Translation2d(
                                                                        Meters.of(0.0),
                                                                        Meters.of(0.0)
                                                                    ),
                                                                    new Translation2d(
                                                                        Meters.of(FieldConstants.fieldLength),
                                                                        Meters.of(FieldConstants.LinesHorizontal.center)
                                                                ));
        
    }
}
