package frc.robot.subsystems.launcher;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.launcher.interpolator.BilinearStrategy;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;

import static edu.wpi.first.units.Units.*;

public class LauncherConstants {
    public static class Launcher {
        public static final double       kS  = 0.4;
        public static final double       kV  = 0.17;
        public static final PIDConstants PID = new PIDConstants(0.15, 0.05, 0);

        public static final double SENSOR_RATIO  = 1.0 / 1.5;
        public static final int    CURRENT_LIMIT = 60;

        public static final LaunchStrategy DEFAULT_LAUNCH_STRATEGY = new BilinearStrategy();

        public static final AngularVelocity LAUNCHER_IDLE_SPEED = RotationsPerSecond.of(40);

        public static final Distance ROLLER_DIAMETER = Inches.of(4);

        public static final Distance ROLLER_CIRCUMFERENCE = ROLLER_DIAMETER.times(Math.PI);
    }

    public static class Hood {
        /**
         * imposed max extension of hood servo to prevent overextension.
         */
        public static final Distance MAX_EXTENSION = Millimeters.of(140);

        public static final int MAX_PULSE_WIDTH       = 2000;
        public static final int SERVO_DEADBAND_MAX    = 1800;
        public static final int SERVO_DEADBAND_CENTER = 1500;
        public static final int SERVO_DEADBAND_MIN    = 1200;
        public static final int MIN_PULSE_WIDTH       = 1000;

        public static final int    HOOD_INVALIDATION_THRESHOLD_MM = 5;
        public static final double HOOD_INVALIDATION_POLL_SECONDS = 0.5;
    }
}
