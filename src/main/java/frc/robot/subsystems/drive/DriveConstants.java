package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;

import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class DriveConstants {
    public static final Mass ROBOT_FULL_MASS = Kilograms.of(62.319);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(2.881);
    public static final double WHEEL_COF = 1.916;
}
