package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class DriveConstants {
    public static final Mass ROBOT_FULL_MASS = Kilograms.of(62.319);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(2.881);
    public static final double WHEEL_COF = 1.916;
    public static final double ODOMETRY_VELOCITY_UPDATE_FREQUENCY = 50;
    public static final Distance ROBOT_WIDTH = Inches.of(27.25);
    public static final Distance BUMPER_DEPTH = Meters.of(Units.inchesToMeters(2.25));
}
