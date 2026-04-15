package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

/**
 * Interpolates a {@link LaunchConfig} (angular velocity and shoot hoodExtension) given a displacement to shoot the
 * fuel.
 * <p>
 * The angular velocity and launch hoodExtension are separated into 2 different functions, only correlated by the common
 * independent variable (distance).
 *
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class BilinearStrategy extends LaunchStrategy {
    private static final InterpolatingDoubleTreeMap HOOD_INTERPOLATOR     = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap VELOCITY_INTERPOLATOR = new InterpolatingDoubleTreeMap();

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        return new LaunchConfig(
                Millimeters.of(HOOD_INTERPOLATOR.get(displacement.in(Meters))),
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
     * @param hoodExt  extension of the hood when the fuel was shot
     * @param speed    angular velocity that the fuel was shot at
     * @param distance total distance the fuel traveled
     */
    private static void addData(Distance hoodExt, AngularVelocity speed, Distance distance) {
        HOOD_INTERPOLATOR.put(distance.in(Meters), hoodExt.in(Millimeters));
        VELOCITY_INTERPOLATOR.put(distance.in(Meters), speed.in(RotationsPerSecond));
    }

    static {
        // PREVIOUS TUNING FEB/MAR
//        addData(Millimeters.of(45), RotationsPerSecond.of(40), Meters.of(1.83));
//        addData(Millimeters.of(50), RotationsPerSecond.of(42), Meters.of(2.310));
//        addData(Millimeters.of(65), RotationsPerSecond.of(43.5), Meters.of(2.802));
//        addData(Millimeters.of(70), RotationsPerSecond.of(45.5), Meters.of(3.16));
//        addData(Millimeters.of(75), RotationsPerSecond.of(54), Meters.of(4.730));
//        addData(Millimeters.of(75), RotationsPerSecond.of(57), Meters.of(5.006));

        // ONCMP TUNING APR 14 2026
        addData(Millimeters.of(45), RotationsPerSecond.of(39.5), Meters.of(1.7));
        addData(Millimeters.of(60), RotationsPerSecond.of(43.5), Meters.of(2.81));
        addData(Millimeters.of(68), RotationsPerSecond.of(57), Meters.of(4.5));
    }
}
