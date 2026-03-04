package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

/**
 * Interpolates a {@link LaunchConfig} (angular velocity and shoot angle) given a displacement to shoot the fuel.
 * <p>
 * The angular velocity and launch angle are separated into 2 different functions, only correlated by the common
 * independent variable (distance).
 *
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class BilinearStrategy extends LaunchStrategy {
    private static final InterpolatingDoubleTreeMap ANGLE_INTERPOLATOR    = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap VELOCITY_INTERPOLATOR = new InterpolatingDoubleTreeMap();

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        return new LaunchConfig(
                Radians.of(ANGLE_INTERPOLATOR.get(displacement.in(Meters))),
                RotationsPerSecond.of(VELOCITY_INTERPOLATOR.get(displacement.in(Meters)))
        );
    }

    @Override
    public String getName() {
        return "Bilinear Interpolation";
    }

    /**
     * Adds a test point to the 2 tree maps used internally by the data interpolator
     *
     * @param angle    angle that the fuel was shot at
     * @param speed    angular velocity that the fuel was shot at
     * @param distance total distance the fuel traveled
     */
    private static void addData(Angle angle, AngularVelocity speed, Distance distance) {
        ANGLE_INTERPOLATOR.put(distance.in(Meters), angle.in(Radians));
        VELOCITY_INTERPOLATOR.put(distance.in(Meters), speed.in(RotationsPerSecond));
    }

    static {
        // ==== TESTING DATA FOR PROTOTYPE LAUNCHER ====
        addData(Degrees.of(30), RotationsPerSecond.of(46), Meters.of(2.0975));
        addData(Degrees.of(37), RotationsPerSecond.of(54.5), Meters.of(4.0375));
    }
}
