package frc.robot.subsystems.launcher;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import frc.robot.subsystems.launcher.interpolator.BilinearStrategy;
import frc.robot.subsystems.launcher.interpolator.LaunchStrategy;

import static edu.wpi.first.units.Units.*;

public class LauncherConstants {
	public static class Launcher {
		public static final double kS = 0.4;
		public static final double kV = 0.17; //0.17
//0.1
		public static final PIDConstants PID = new PIDConstants(0.15, 0.05, 0);


		public static final double MOTOR_ENCODER_GEAR_RATIO = 1.0 / 1.5;
		public static final int SUPPLY_CURRENT_LIMIT = 60;

		public static final AngularVelocity LAUNCH_SPEED_OFFSET_INCREMENT = RPM.of(1);
		public static final LaunchStrategy DEFAULT_LAUNCH_STRATEGY = new BilinearStrategy();

        public static final AngularVelocity LAUNCHER_IDLE_SPEED = RotationsPerSecond.of(30);
	}

	public static class Hood {
		/**
		 * derived from linear map of deg to mm
		 */
		public static final Per<DistanceUnit, AngleUnit> MM_PER_DEG = Millimeters.per(Degrees).ofNative(3.37591);
		/**
		 * derived from linear map of deg to mm
		 */
		public static final Distance OFFSET_MM = Millimeters.of(51.27737);

		/**
		 * stored as raw number (degrees) to save computation frames
		 */
//		3.80952
        public static final double MIN_ANGLE_DEG = 18;
		public static final double MAX_ANGLE_DEG = MIN_ANGLE_DEG + 30;

		/**
		 * imposed max extension of hood servo to prevent overextension.
		 */
		public static final Distance MAX_EXTENSION = Millimeters.of(140);

		public static final int MAX_PULSE_WIDTH = 2000;
		public static final int SERVO_DEADBAND_MAX = 1800;
		public static final int SERVO_DEADBAND_CENTER = 1500;
		public static final int SERVO_DEADBAND_MIN = 1200;
		public static final int MIN_PULSE_WIDTH = 1000;
	}

	public static class Ultrasonic {
		public static final Per<DistanceUnit, VoltageUnit> MM_PER_VOLT = Millimeters.of(1024.0)
				.div(Volts.of(5.0));
	}
}
