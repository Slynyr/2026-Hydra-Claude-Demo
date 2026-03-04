package frc.robot.util;

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
}
