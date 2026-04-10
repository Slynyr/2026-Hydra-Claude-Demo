package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.*;

/**
 * Basic math utility functions for use throughout robot code
 *
 * @author Logan Dhillon
 */
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

    /**
     * Calculates rotation difference for rotation2d
     *
     * @param a 1st rotation2d
     * @param b 2nd rotation2d
     *
     * @return angle between them
     */
    public static Angle rotationDifference(Rotation2d a, Rotation2d b) {
        double difference = a.getRadians() - b.getRadians();

        difference = (difference + Math.PI) % (2 * Math.PI) - Math.PI;

        while (difference < -Math.PI)
            difference += 2 * Math.PI;

        while (difference > Math.PI)
            difference -= 2 * Math.PI;

        return Radians.of(Math.abs(difference));
    }

    /**
     * Calculates linear surface velocity from a rotational velocity given the circumference of the mechanism.
     *
     * @return surface speed
     */
    public static LinearVelocity calculateSurfaceSpeed(AngularVelocity speed, Distance mechCircumference) {
        return MetersPerSecond.of(speed.in(RotationsPerSecond) * mechCircumference.in(Meters));
    }

    /**
     * Calculates rotational velocity from a linear surface velocity given the circumference of the mechanism.
     *
     * @return surface speed
     */
    public static AngularVelocity calculateAngularVelocity(LinearVelocity speed, Distance mechCircumference) {
        return RotationsPerSecond.of(speed.in(MetersPerSecond) / mechCircumference.in(Meters));
    }
}
