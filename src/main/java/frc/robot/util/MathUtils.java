package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class MathUtils {
    /**
     * Calculates if a value is within another value, given a percentage tolerance
     *
     * @param actual  value to measure
     * @param target  value to check against
     * @param percent tolerance %
     *
     * @return if value is within tolerance
     */
    public static boolean withinTolerance(double actual, double target, double percent) {
        return Math.abs(actual - target) <= Math.abs(target) * (percent / 100.0);
    }

    /**
     * Checks if an angle (a) is near another angle (b) with a given tolernace, accounting for the unit circle.
     *
     * @param actual    actual angle of the system (a)
     * @param target    target/expected angle of the system (b)
     * @param tolerance allowed tolerance in degrees
     *
     * @return if difference in angles is less than the tolerance
     */
    public static boolean isAngleNear(Rotation2d actual, Rotation2d target, Angle tolerance) {
        double diff = Math.abs(actual.getDegrees() - target.getDegrees()) % 360;
        if (diff > 180) diff = 360 - diff;
        return diff <= tolerance.in(Degrees);
    }
}
