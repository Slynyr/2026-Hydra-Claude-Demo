package frc.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public final class DriveConstants {
    public static final double ODOMETRY_VELOCITY_UPDATE_FREQUENCY = 50;

    public static final Mass     ROBOT_FULL_MASS = Kilograms.of(62.319);
    public static final double   WHEEL_COF       = 1.916;
    public static final Distance ROBOT_WIDTH     = Inches.of(27.25);
    public static final Distance BUMPER_DEPTH    = Meters.of(Units.inchesToMeters(2.25));

    // PathPlanner config constants
    public static final double ROBOT_MASS_KG  = 74.088;
    public static final double ROBOT_MOI      = 6.883;
    public static final double AUTO_WHEEL_COF = 1.2;

    public static final Current DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(50);
    public static final Current TURN_SUPPLY_CURRENT_LIMIT = Amps.of(25);
    public static final Current TURN_STATOR_CURRENT_LIMIT = Amps.of(60);
}