package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Angle;
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
 * @apiNote This strategy internally uses a {@link BilinearStrategy}; the test points in the {@link BilinearStrategy}
 * will be used here.
 * @deprecated This strategy was ruled performatively impractical and will not be updated since Feb. 2026.
 */
public class DynamicHoodBilinearStrategy extends BilinearStrategy {
    private static final Angle ANGLE_ADJUSTMENT = Degrees.of(5);

    private LaunchConfig lastConfig;

    /**
     * Tuned value that affects the correction rate of the hood as per the velocity error.
     */
    private static final float ALPHA = 0.005f;

    /**
     * Computes the new hood value to correct the error of theoretical velocity and real velocity
     *
     * @param targetVelocity target/theoretical velocity
     * @param realVelocity   actual velocity of the motor
     * @param hoodAngle      measured hood hoodExtension
     *
     * @return new hood hoodExtension
     */
    public Angle computeHoodAdjustment(
            AngularVelocity targetVelocity,
            AngularVelocity realVelocity,
            Angle hoodAngle,
            Angle targetAngle) {
        return Radians.of(
                ALPHA
                * targetVelocity.minus(realVelocity).in(RadiansPerSecond)
                * Math.cos(2 * hoodAngle.minus(targetAngle.plus(ANGLE_ADJUSTMENT)).in(Radians)));
    }

    @Override
    public LaunchConfig interpolate(Distance displacement) {
        var params = super.interpolate(displacement);
        lastConfig = params;
        return params;
    }

    @Override
    public void periodic() {
//        if (lastConfig == null) return;
//
//        // update hood before returning interpolation
//        Angle err = computeHoodAdjustment(
//                lastConfig.speed(),
//                this.launcher.getVelocity(),
//                this.launcher.getHoodAngle(),
//                lastConfig.hoodExtension()
//        );
//        Logger.recordOutput("Launcher/Interpolator/DynamicHoodAdjustment", err);
//        CommandScheduler.getInstance().schedule(this.launcher.setHoodAngle(() -> lastConfig.hoodExtension().plus
//        (err)));
    }

    @Override
    public String getName() {
        return "Bilinear w/ Dynamic Hood";
    }
}
