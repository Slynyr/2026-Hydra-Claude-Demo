package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.launcher.Launcher;

public abstract class LaunchStrategy {
    protected Launcher launcher;

    /**
     * Interpolates the fastest angular velocity and shoot hoodExtension for the launcher based on the displacement to fire the
     * fuel.
     *
     * @param displacement total straight-line displacement to shoot fuel at
     *
     * @return {@link LaunchConfig}, containing angular velocity and hoodExtension to shoot at
     *
     * @apiNote Uses the active {@link LaunchStrategy}
     */
    public abstract LaunchConfig interpolate(Distance displacement);

    /**
     * Called in periodic of the parent {@link Launcher} subsystem
     */
    public void periodic() {}

    public abstract String getName();

    /**
     * Gets a list of all the available {@link LaunchStrategy} that the user can choose from.
     */
    public static LaunchStrategy[] getLaunchStrategies() {
        return new LaunchStrategy[]{
                new BilinearStrategy(),
                new DynamicHoodBilinearStrategy()
        };
    }

    public void setLauncher(Launcher launcher) {
        if (this.launcher != null) throw new IllegalStateException("Launcher is already set!");
        this.launcher = launcher;
    }
}
