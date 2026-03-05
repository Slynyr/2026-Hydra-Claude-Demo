package frc.robot.subsystems.intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;
import frc.robot.Constants.DeviceID;

public final class IntakeConstants {

	public static final class Extension {
		public static final Distance EXTENSION_MIN_DISTANCE = Meters.of(0.0);
		public static final Distance EXTENSION_DISTANCE = Meters.of(0.2794);
		public static final Distance EXTENSION_MAX_DISTANCE = Meters.of(0.289146);

		public static final PIDController PID = new PIDController(1.0,0.0,0.0);
		public static final PIDConstants TALONFX_PID = new PIDConstants(PID.getP(), PID.getI(), PID.getD());

		public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
		public static final Current MAX_CURRENT = Amps.of(30.0);
		public static final double GEARING = 9.0/1.0;
		public static final Mass INTAKE_MASS = Kilograms.of(3.1818);

		public static final boolean INTAKE_IS_TUNING = false;

        public static final Distance RETRACT_INCREMENT = Meters.of(0.0254);
		public static final Distance EXTEND_INCREMENT = Meters.of(0.0127);
		public static final Distance INITIAL_SETPOINT = Meters.of(0.254);

		public static final Distance KILLSWITCH_TOLERANCE = Meters.of(0.0508);

		public static final Time WAIT_TIME = Seconds.of(0.01);

		public static final int UPDATE_FREQUENCY = 50; // in Hz

		public static final Current CRASH_CURRENT_THRESHOLD = Amps.of(50.0); //TODO: test it

	}

    public static final class Roller {
        public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
        public static final Current MAX_CURRENT = Amps.of(30.0);
        public static final double GEARING = 1.5/1.0;
        public static final Mass ROLLER_MASS = Kilograms.of(0.0813636);
    }
}