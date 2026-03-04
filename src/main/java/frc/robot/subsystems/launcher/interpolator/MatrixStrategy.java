package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Arrays;

import static edu.wpi.first.units.Units.*;

/**
 * Interpolates a {@link LaunchConfig} (angular velocity and shoot angle) given a displacement to shoot the fuel.
 *
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class MatrixStrategy extends LaunchStrategy {
    /**
     * map of tested shots keyed by distance to travel, storing 2x1 matrices [angle (rad), velocity (rps)]
     */
    private static final InterpolatingMatrixTreeMap<Double, N2, N1> INTERPOLATOR = new InterpolatingMatrixTreeMap<>();

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        Matrix<N2, N1> interpolated = INTERPOLATOR.get(displacement.in(Meters));

        return new LaunchConfig(
                Radians.of(interpolated.get(0, 0)),
                RotationsPerSecond.of(interpolated.get(1, 0))
        );
    }

    @Override
    public String getName() {
        return "3D Matrix Interpolation";
    }

    /**
     * Adds a test point to the {@link InterpolatingMatrixTreeMap} used internally by the data interpolator
     *
     * @param angle  angle that the fuel was shot at
     * @param speed  angular velocity that the fuel was shot at
     * @param trials total displacement traveled, ideally use 3 trials
     */
    private static void addData(Angle angle, AngularVelocity speed, Distance... trials) {
        Matrix<N2, N1> matrix = new Matrix<>(N2.instance, N1.instance);
        matrix.set(0, 0, angle.in(Radians));
        matrix.set(1, 0, speed.in(RotationsPerSecond));
        INTERPOLATOR.put(Arrays.stream(trials).mapToDouble(d -> d.in(Meters)).average().orElseThrow(), matrix);
    }

    static {
        // ==== TESTING DATA FOR PROTOTYPE LAUNCHER ====
        addData(Degrees.of(75), RotationsPerSecond.of(50), Meters.of(2.30));
        addData(Degrees.of(75), RotationsPerSecond.of(60), Meters.of(3.60));
        addData(Degrees.of(75), RotationsPerSecond.of(70), Meters.of(4.75));
        addData(Degrees.of(75), RotationsPerSecond.of(80), Meters.of(6.25));
        addData(Degrees.of(75), RotationsPerSecond.of(90), Meters.of(7.68));
        addData(Degrees.of(75), RotationsPerSecond.of(100), Meters.of(9.40));
    }
}
